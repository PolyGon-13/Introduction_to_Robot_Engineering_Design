
#include <Arduino.h>

#define PI_F 3.1416f

const byte PWMPin_r = 9;
const byte DirPin1_r = 10;
const byte DirPin2_r = 11;
const byte ENC_A_r = 2;
const byte ENC_B_r = 4;

const byte PWMPin_l = 6;
const byte DirPin1_l = 7;
const byte DirPin2_l = 8;
const byte ENC_A_l = 3;
const byte ENC_B_l = 5;

const bool INVERT_ENC_R = true;
const bool INVERT_ENC_L = false;
const bool REVERSE_MOTOR_R = true;
const bool REVERSE_MOTOR_L = true;

const float WHEEL_R = 0.034f;
const float WHEEL_BASE = 0.179f;
const float PPR = 1012.0f;
const float COUNT_PER_RAD = PPR / (2.0f * PI_F);

const float V_MAX = 6.0f;
const float V_MIN = -V_MAX;
const float DRIVER_DEADBAND_V = 0.05f;
const float WHEEL_SPEED_MAX = 8.0f;

struct WheelPID {
  float kp, ki, kd;
  float integ, prev_e;
  float target;
  float meas;
};
WheelPID pidR = {0.50f, 1.50f, 0.0f, 0, 0, 0, 0};
WheelPID pidL = {0.50f, 1.50f, 0.0f, 0, 0, 0, 0};
const float WHEEL_FF = 1.0f;

float V_cmd = 0.0f;
float W_cmd = 0.0f;
bool pivotMode = false;

const unsigned long PID_INTERVAL_MS = 20;
const unsigned long CMD_TIMEOUT_MS = 300;
const unsigned long ODOM_INTERVAL_MS = 50;
unsigned long lastPidMs = 0;
unsigned long lastCmdMs = 0;
unsigned long lastOdomMs = 0;

volatile long EncoderCount_r = 0;
volatile long EncoderCount_l = 0;
long encR_prev = 0, encL_prev = 0;

void ISR_Encoder_A_r() {
  bool a = digitalRead(ENC_A_r), b = digitalRead(ENC_B_r);
  int delta = (a == b) ? 1 : -1;
  if (INVERT_ENC_R) delta = -delta;
  EncoderCount_r += delta;
}
void ISR_Encoder_A_l() {
  bool a = digitalRead(ENC_A_l), b = digitalRead(ENC_B_l);
  int delta = (a == b) ? 1 : -1;
  if (INVERT_ENC_L) delta = -delta;
  EncoderCount_l += delta;
}

void resetEncoderCounts() {
  noInterrupts();
  EncoderCount_r = 0;
  EncoderCount_l = 0;
  interrupts();
  encR_prev = 0;
  encL_prev = 0;
}

static inline void writeDriver_r(float V) {
  if (fabs(V) < DRIVER_DEADBAND_V) V = 0.0f;
  V = constrain(V, V_MIN, V_MAX);
  int pwm = constrain((int)(255.0f * fabs(V) / V_MAX), 0, 255);
  if (V == 0.0f) {
    digitalWrite(DirPin1_r, LOW);
    digitalWrite(DirPin2_r, LOW);
  } else {
    bool forward = (V > 0) ^ REVERSE_MOTOR_R;
    digitalWrite(DirPin1_r, forward ? HIGH : LOW);
    digitalWrite(DirPin2_r, forward ? LOW : HIGH);
  }
  analogWrite(PWMPin_r, pwm);
}
static inline void writeDriver_l(float V) {
  if (fabs(V) < DRIVER_DEADBAND_V) V = 0.0f;
  V = constrain(V, V_MIN, V_MAX);
  int pwm = constrain((int)(255.0f * fabs(V) / V_MAX), 0, 255);
  if (V == 0.0f) {
    digitalWrite(DirPin1_l, LOW);
    digitalWrite(DirPin2_l, LOW);
  } else {
    bool forward = (V > 0) ^ REVERSE_MOTOR_L;
    digitalWrite(DirPin1_l, forward ? HIGH : LOW);
    digitalWrite(DirPin2_l, forward ? LOW : HIGH);
  }
  analogWrite(PWMPin_l, pwm);
}

void resetWheelPID(WheelPID &p) {
  p.integ = 0.0f;
  p.prev_e = 0.0f;
  p.target = 0.0f;
}

void stopMotion() {
  V_cmd = 0.0f;
  W_cmd = 0.0f;
  pivotMode = false;
  resetWheelPID(pidR);
  resetWheelPID(pidL);
  writeDriver_r(0.0f);
  writeDriver_l(0.0f);
}

float computePID(WheelPID &p, float dt) {
  float e = p.target - p.meas;
  p.integ += e * dt;
  p.integ = constrain(p.integ, -3.0f, 3.0f);
  float d = (e - p.prev_e) / dt;
  p.prev_e = e;
  return WHEEL_FF * p.target + (p.kp * e + p.ki * p.integ + p.kd * d);
}

void resolveWheelTargets(float v, float w) {
  if (pivotMode) {
    float spd = constrain(WHEEL_BASE * fabs(w) / WHEEL_R, 0.0f, WHEEL_SPEED_MAX);
    if (w >= 0.0f) {
      pidL.target = -spd;
      pidR.target = 0.0f;
    } else {
      pidR.target = -spd;
      pidL.target = 0.0f;
    }
    return;
  }
  float wL = (v - WHEEL_BASE * w * 0.5f) / WHEEL_R;
  float wR = (v + WHEEL_BASE * w * 0.5f) / WHEEL_R;
  pidL.target = constrain(wL, -WHEEL_SPEED_MAX, WHEEL_SPEED_MAX);
  pidR.target = constrain(wR, -WHEEL_SPEED_MAX, WHEEL_SPEED_MAX);
}

String inputPi = "";

void processCommand(String s) {
  s.trim();
  if (s.length() == 0) return;
  char c0 = s.charAt(0);
  if (c0 == 'S' || c0 == 's') {
    stopMotion();
    lastCmdMs = millis();
  } else if (c0 == 'R' || c0 == 'r') {
    resetEncoderCounts();
    Serial1.println(F("RESET"));
    Serial.println(F("RESET"));
  } else if (c0 == 'V' || c0 == 'v') {
    int comma = s.indexOf(',');
    if (comma <= 1 || comma >= s.length() - 1) return;
    V_cmd = s.substring(1, comma).toFloat();
    W_cmd = s.substring(comma + 1).toFloat();
    pivotMode = false;
    lastCmdMs = millis();
  } else if (c0 == 'P' || c0 == 'p') {
    if (s.length() < 2) return;
    V_cmd = 0.0f;
    W_cmd = s.substring(1).toFloat();
    pivotMode = true;
    lastCmdMs = millis();
  }
}

void readRPSerial() {
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

void sendOdomSerial(unsigned long now, long enc_l, long enc_r) {
  Serial1.print(F("O,"));
  Serial1.print(enc_l);
  Serial1.print(',');
  Serial1.print(enc_r);
  Serial1.print(',');
  Serial1.println(now);
}

void setup() {
  Serial.begin(115200);
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
  writeDriver_r(0);
  writeDriver_l(0);

  lastPidMs = millis();
  lastCmdMs = millis();
  lastOdomMs = millis();
  Serial.println(F("[READY] project3 velocity controller"));
}

void loop() {
  readRPSerial();

  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) stopMotion();

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

  if (V_cmd == 0.0f && W_cmd == 0.0f) {
    stopMotion();
  } else {
    resolveWheelTargets(V_cmd, W_cmd);
    writeDriver_r(computePID(pidR, dt));
    writeDriver_l(computePID(pidL, dt));
  }

  if (now - lastOdomMs >= ODOM_INTERVAL_MS) {
    lastOdomMs = now;
    sendOdomSerial(now, enc_l, enc_r);
  }
}


