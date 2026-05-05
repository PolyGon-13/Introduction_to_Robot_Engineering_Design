#include <Arduino.h>

#define PI_F 3.1416f

const byte PWMPin_r  = 9;
const byte DirPin1_r = 10;
const byte DirPin2_r = 11;
const byte ENC_A_r   = 2;
const byte ENC_B_r   = 4;

const byte PWMPin_l  = 6;
const byte DirPin1_l = 7;
const byte DirPin2_l = 8;
const byte ENC_A_l   = 3;
const byte ENC_B_l   = 5;

const bool INVERT_ENC_R    = true;
const bool INVERT_ENC_L    = false;
const bool REVERSE_MOTOR_R = true;
const bool REVERSE_MOTOR_L = true;

const float WHEEL_R       = 0.034f;          // [m] 휠 반경
const float WHEEL_BASE    = 0.179f;          // [m] **실측치 반영**
const float PPR           = 1012.0f;
const float COUNT_PER_RAD = PPR / (2.0f * PI_F);

const float V_MAX = 6.0f;
const float V_MIN = -6.0f;
const float DRIVER_DEADBAND_V = 0.05f;

const float WHEEL_SPEED_MAX = 8.0f;          // [rad/s]

struct WheelPID {
  float kp, ki, kd;
  float integ, prev_e;
  float target;   // [rad/s]
  float meas;     // [rad/s]
};
WheelPID pidR = {0.50f, 1.50f, 0.0f, 0, 0, 0, 0};
WheelPID pidL = {0.50f, 1.50f, 0.0f, 0, 0, 0, 0};

const float L_SCALE   = 1.00f;
const float R_SCALE   = 1.00f;
const float WHEEL_FF  = 1.0f;                // [V/(rad/s)] 피드포워드

// ────────────── 명령/타이밍 ──────────────
float V_cmd = 0.0f;
float W_cmd = 0.0f;

const unsigned long PID_INTERVAL_MS = 20;
const unsigned long CMD_TIMEOUT_MS  = 3000;

unsigned long lastPidMs = 0;
unsigned long lastCmdMs = 0;

// ────────────── 엔코더 ──────────────
volatile long EncoderCount_r = 0;
volatile long EncoderCount_l = 0;
long encR_prev = 0, encL_prev = 0;

void ISR_Encoder_A_r() {
  bool pinA = digitalRead(ENC_A_r);
  bool pinB = digitalRead(ENC_B_r);
  int delta;
  if (pinA == HIGH) delta = (pinB == LOW) ? -1 : 1;
  else              delta = (pinB == HIGH) ? -1 : 1;
  if (INVERT_ENC_R) delta = -delta;
  EncoderCount_r += delta;
}
void ISR_Encoder_A_l() {
  bool pinA = digitalRead(ENC_A_l);
  bool pinB = digitalRead(ENC_B_l);
  int delta;
  if (pinA == HIGH) delta = (pinB == LOW) ? -1 : 1;
  else              delta = (pinB == HIGH) ? -1 : 1;
  if (INVERT_ENC_L) delta = -delta;
  EncoderCount_l += delta;
}

// ────────────── 모터 출력 ──────────────
static inline void writeDriver_r(float V) {
  if (fabs(V) < DRIVER_DEADBAND_V) V = 0.0f;
  V = constrain(V, V_MIN, V_MAX);
  int PWMval = constrain((int)(255.0f * fabs(V) / V_MAX), 0, 255);
  if (V == 0.0f) {
    digitalWrite(DirPin1_r, LOW);
    digitalWrite(DirPin2_r, LOW);
  } else {
    bool forward = (V > 0) ^ REVERSE_MOTOR_R;
    digitalWrite(DirPin1_r, forward ? HIGH : LOW);
    digitalWrite(DirPin2_r, forward ? LOW : HIGH);
  }
  analogWrite(PWMPin_r, PWMval);
}
static inline void writeDriver_l(float V) {
  if (fabs(V) < DRIVER_DEADBAND_V) V = 0.0f;
  V = constrain(V, V_MIN, V_MAX);
  int PWMval = constrain((int)(255.0f * fabs(V) / V_MAX), 0, 255);
  if (V == 0.0f) {
    digitalWrite(DirPin1_l, LOW);
    digitalWrite(DirPin2_l, LOW);
  } else {
    bool forward = (V > 0) ^ REVERSE_MOTOR_L;
    digitalWrite(DirPin1_l, forward ? HIGH : LOW);
    digitalWrite(DirPin2_l, forward ? LOW : HIGH);
  }
  analogWrite(PWMPin_l, PWMval);
}

void resetWheelPID(WheelPID &p) {
  p.integ = 0.0f;
  p.prev_e = 0.0f;
  p.target = 0.0f;
}

void stopMotion() {
  V_cmd = 0.0f;
  W_cmd = 0.0f;
  resetWheelPID(pidR);
  resetWheelPID(pidL);
  writeDriver_r(0.0f);
  writeDriver_l(0.0f);
}

// ────────────── PID 한 스텝 ──────────────
float computePID(WheelPID &p, float dt) {
  float e = p.target - p.meas;
  p.integ += e * dt;
  p.integ = constrain(p.integ, -3.0f, 3.0f);
  float d = (e - p.prev_e) / dt;
  p.prev_e = e;
  float V = WHEEL_FF * p.target + p.kp * e + p.ki * p.integ + p.kd * d;
  return V;
}

// ────────────── V,W → 좌/우 휠 각속도 분해 ──────────────
void resolveWheelTargets(float v, float w) {
  float wL = (v - WHEEL_BASE * w * 0.5f) / WHEEL_R;
  float wR = (v + WHEEL_BASE * w * 0.5f) / WHEEL_R;
  pidL.target = constrain(wL, -WHEEL_SPEED_MAX, WHEEL_SPEED_MAX);
  pidR.target = constrain(wR, -WHEEL_SPEED_MAX, WHEEL_SPEED_MAX);
}

// ────────────── 명령 파싱 ──────────────
String inputPi  = "";
String inputUsb = "";

void processCommand(String s) {
  s.trim();
  if (s.length() == 0) return;
  char c0 = s.charAt(0);
  if (c0 == 'S' || c0 == 's') {
    stopMotion();
    lastCmdMs = millis();
  } else if (c0 == 'V' || c0 == 'v') {
    int comma = s.indexOf(',');
    if (comma > 1) {
      V_cmd = s.substring(1, comma).toFloat();
      W_cmd = s.substring(comma + 1).toFloat();
      lastCmdMs = millis();
    }
  } else if (c0 == 'G' || c0 == 'g') {
    // 게인 출력
    Serial.print(F("[GAIN] kp=")); Serial.print(pidR.kp, 3);
    Serial.print(F(" ki=")); Serial.print(pidR.ki, 3);
    Serial.print(F(" kd=")); Serial.print(pidR.kd, 3);
    Serial.print(F(" FF=")); Serial.println(WHEEL_FF, 3);
  }
}

void readSerial1Line() {
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n' || c == '\r') {
      if (inputPi.length() > 0) {
        processCommand(inputPi);
        inputPi = "";
      }
    } else if (inputPi.length() < 32) {
      inputPi += c;
    }
  }
}

void readSerialLine() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputUsb.length() > 0) {
        Serial.print(F("[USB] cmd: ")); Serial.println(inputUsb);
        processCommand(inputUsb);
        inputUsb = "";
      }
    } else if (inputUsb.length() < 32) {
      inputUsb += c;
    }
  }
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  pinMode(ENC_A_r, INPUT_PULLUP);
  pinMode(ENC_B_r, INPUT_PULLUP);
  pinMode(ENC_A_l, INPUT_PULLUP);
  pinMode(ENC_B_l, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A_r), ISR_Encoder_A_r, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_l), ISR_Encoder_A_l, CHANGE);

  pinMode(PWMPin_r, OUTPUT);
  pinMode(DirPin1_r, OUTPUT);
  pinMode(DirPin2_r, OUTPUT);
  pinMode(PWMPin_l, OUTPUT);
  pinMode(DirPin1_l, OUTPUT);
  pinMode(DirPin2_l, OUTPUT);

  inputPi.reserve(40);
  inputUsb.reserve(40);

  writeDriver_r(0);
  writeDriver_l(0);

  lastPidMs = millis();
  lastCmdMs = millis();
  Serial.println(F("[READY] maze_run V,W controller"));
  Serial.println(F("Cmd: 'V0.20,0.0', 'V0,1.0', 'S', 'G'(gain)"));
}

void loop() {
  readSerial1Line();
  readSerialLine();

  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    stopMotion();
  }

  unsigned long now = millis();
  if (now - lastPidMs < PID_INTERVAL_MS) return;
  float dt = (now - lastPidMs) / 1000.0f;
  if (dt < 0.001f) dt = 0.001f;
  lastPidMs = now;

  long enc_r, enc_l;
  noInterrupts();
  enc_r = EncoderCount_r;
  enc_l = EncoderCount_l;
  interrupts();
  long dR = enc_r - encR_prev; encR_prev = enc_r;
  long dL = enc_l - encL_prev; encL_prev = enc_l;
  pidR.meas = (dR / COUNT_PER_RAD) / dt;
  pidL.meas = (dL / COUNT_PER_RAD) / dt;

  resolveWheelTargets(V_cmd, W_cmd);

  float Vr = 0.0f;
  float Vl = 0.0f;
  if (V_cmd == 0.0f && W_cmd == 0.0f) {
    stopMotion();
  } else {
    Vr = R_SCALE * computePID(pidR, dt);
    Vl = L_SCALE * computePID(pidL, dt);

    writeDriver_r(Vr);
    writeDriver_l(Vl);
  }

  static unsigned long lastLogMs = 0;
  if (now - lastLogMs > 200) {
    lastLogMs = now;
    Serial.print(F("V=")); Serial.print(V_cmd, 2);
    Serial.print(F(" W=")); Serial.print(W_cmd, 2);
    Serial.print(F(" | tR=")); Serial.print(pidR.target, 2);
    Serial.print(F(" mR=")); Serial.print(pidR.meas, 2);
    Serial.print(F(" tL=")); Serial.print(pidL.target, 2);
    Serial.print(F(" mL=")); Serial.print(pidL.meas, 2);
    Serial.print(F(" | Vr=")); Serial.print(Vr, 2);
    Serial.print(F(" Vl=")); Serial.println(Vl, 2);
  }
}
