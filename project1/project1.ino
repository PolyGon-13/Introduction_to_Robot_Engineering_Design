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
const bool INVERT_ENC_L = true;

const bool REVERSE_MOTOR_R = true;
const bool REVERSE_MOTOR_L = true;

const float WHEEL_R = 0.034f;
const float WHEEL_L = 0.2f;
const float PPR = 1012.0f;

const float COUNT_PER_M_CAL = 1.0204f;
const float COUNT_PER_M = (PPR / (2.0f * PI_F * WHEEL_R)) * COUNT_PER_M_CAL;
const long STOP_TOL_CNT = 35;

float kp_pos = 0.1f;
float ki_pos = 0.002f;
float kd_pos = 0.000f;

float kp_sync = 0.085f;
float ki_sync = 0.01f;
float kd_sync = 0.000f;

const float STRAIGHT_BIAS_CNT_PER_M = -30.0f;

const float V_MAX = 6.0f;
const float V_MIN = -6.0f;

const unsigned long START_RAMP_MS = 900;
const float STOP_RAMP_M = 0.1f;
const float MIN_RAMP_SCALE = 0.1f;

volatile long EncoderCount_r = 0;
volatile long EncoderCount_l = 0;

enum DriveState { ST_IDLE, ST_RUN, ST_DONE };
DriveState driveState = ST_IDLE;

long startCount_r = 0;
long startCount_l = 0;
long targetCount = 0;
float driveSign = 1.0f;

float e_pos_prev = 0;
float inte_pos = 0;

float e_sync_prev = 0;
float inte_sync = 0;

const unsigned long PID_INTERVAL_MS = 20;
unsigned long lastPidMs = 0;
unsigned long driveStartMs = 0;

String inputPi = "";

void ISR_Encoder_A_r() {
  bool pinA = digitalRead(ENC_A_r);
  bool pinB = digitalRead(ENC_B_r);
  int delta;
  if (pinA == HIGH) delta = (pinB == LOW) ? -1 : 1;
  else delta = (pinB == HIGH) ? -1 : 1;
  if (INVERT_ENC_R) delta = -delta;
  EncoderCount_r += delta;
}

void ISR_Encoder_A_l() {
  bool pinA = digitalRead(ENC_A_l);
  bool pinB = digitalRead(ENC_B_l);
  int delta;
  if (pinA == HIGH) delta = (pinB == LOW) ? -1 : 1;
  else delta = (pinB == HIGH) ? -1 : 1;
  if (INVERT_ENC_L) delta = -delta;
  EncoderCount_l += delta;
}

static inline void writeDriver_r(float V) {
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

void startStraightDrive(float distance_m) {
  if (fabs(distance_m) < 0.001f) {
    Serial.println("[ERR] distance too small");
    return;
  }

  driveSign = (distance_m >= 0.0f) ? 1.0f : -1.0f;

  noInterrupts();
  startCount_r = EncoderCount_r;
  startCount_l = EncoderCount_l;
  interrupts();

  targetCount = (long)(fabs(distance_m) * COUNT_PER_M);

  e_pos_prev = (float)targetCount;
  inte_pos = 0;
  e_sync_prev = 0;
  inte_sync = 0;

  driveStartMs = millis();
  lastPidMs = driveStartMs;
  driveState = ST_RUN;

  Serial.print("[START] d=");
  Serial.print(distance_m, 4);
  Serial.print(" m  target_cnt=");
  Serial.println(targetCount);
}

void stopAll(const char *reason) {
  writeDriver_r(0);
  writeDriver_l(0);
  inte_pos = 0;
  inte_sync = 0;
  driveState = ST_IDLE;

  Serial.print("[STOP] ");
  Serial.println(reason);
}

void processCommand(String s) {
  s.trim();
  if (s.length() == 0) return;

  char c0 = s.charAt(0);

  if (c0 == 'D' || c0 == 'd') {
    float dist = s.substring(1).toFloat();

    if (dist == 0.0f) {
      Serial.println("[ERR] zero distance");
      return;
    }

    startStraightDrive(dist);
  } else if (c0 == 'S' || c0 == 's') {
    stopAll("user");
  } else {
    Serial.println("[ERR] use 'D<m>' or 'S'");
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

  writeDriver_r(0);
  writeDriver_l(0);

  Serial.println(F("[READY] Setup Complete!"));
}

void loop() {
  readSerial1Line();

  unsigned long now = millis();

  if (now - lastPidMs < PID_INTERVAL_MS) return;

  float dt_s = (now - lastPidMs) / 1000.0f;
  if (dt_s < 0.001f) dt_s = 0.001f;
  lastPidMs = now;

  long enc_r;
  long enc_l;

  noInterrupts();
  enc_r = EncoderCount_r;
  enc_l = EncoderCount_l;
  interrupts();

  if (driveState == ST_IDLE || driveState == ST_DONE) {
    writeDriver_r(0);
    writeDriver_l(0);
    return;
  }

  long progR = labs(enc_r - startCount_r);
  long progL = labs(enc_l - startCount_l);
  long progAvg = (progR + progL) / 2;

  float e_pos = (float)(targetCount - progAvg);

  if (e_pos <= (float)STOP_TOL_CNT) {
    writeDriver_r(0);
    writeDriver_l(0);
    driveState = ST_DONE;

    Serial.print("DONE R=");
    Serial.print(progR);
    Serial.print(" L=");
    Serial.print(progL);
    Serial.print(" AVG=");
    Serial.println(progAvg);

    return;
  }

  inte_pos += e_pos * dt_s;
  inte_pos = constrain(inte_pos, -20000.0f, 20000.0f);

  float d_pos = (e_pos - e_pos_prev) / dt_s;
  float V_base_raw = kp_pos * e_pos + ki_pos * inte_pos + kd_pos * d_pos;

  float Vcap = 4.0f;

  if (e_pos < 1200.0f) {
    Vcap = 2.0f + 2.0f * (e_pos / 1200.0f);
  }

  Vcap = constrain(Vcap, 2.0f, 4.0f);

  float V_base_target = constrain(V_base_raw * driveSign, -Vcap, Vcap);

  float startRamp = 1.0f;
  if (START_RAMP_MS > 0) {
    startRamp = constrain((float)(now - driveStartMs) / (float)START_RAMP_MS, 0.0f, 1.0f);
  }

  float stopRamp = 1.0f;
  if (STOP_RAMP_M > 0.0f) {
    stopRamp = constrain(e_pos / (STOP_RAMP_M * COUNT_PER_M), MIN_RAMP_SCALE, 1.0f);
  }

  float V_base = V_base_target * min(startRamp, stopRamp);

  e_pos_prev = e_pos;

  float progressRatio = constrain((float)progAvg / (float)targetCount, 0.0f, 1.0f);
  float targetDiff = -75.0f * progressRatio;
  float e_sync = (float)(progR - progL) - targetDiff;

  inte_sync += e_sync * dt_s;
  inte_sync = constrain(inte_sync, -2000.0f, 2000.0f);

  float d_sync = (e_sync - e_sync_prev) / dt_s;
  float V_sync = kp_sync * e_sync + ki_sync * inte_sync + kd_sync * d_sync;

  e_sync_prev = e_sync;

  float V_sync_directed = V_sync * driveSign;

  float V_r = constrain(V_base - V_sync_directed, V_MIN, V_MAX);
  float V_l = constrain(V_base + V_sync_directed, V_MIN, V_MAX);

  writeDriver_r(V_r);
  writeDriver_l(V_l);

  static unsigned long lastLogMs = 0;

  if (now - lastLogMs > 200) {
    lastLogMs = now;

    Serial.print(F("prog="));
    Serial.print(progAvg);
    Serial.print(F(" e_pos="));
    Serial.print(e_pos, 0);
    Serial.print(F(" Vb="));
    Serial.print(V_base, 2);
    Serial.print(F(" R="));
    Serial.print(progR);
    Serial.print(F(" L="));
    Serial.print(progL);
    Serial.print(F(" targetDiff="));
    Serial.print(targetDiff, 1);
    Serial.print(F(" e_sync="));
    Serial.print(e_sync, 0);
    Serial.print(F(" Vs="));
    Serial.print(V_sync, 3);
    Serial.print(F(" Vr="));
    Serial.print(V_r, 2);
    Serial.print(F(" Vl="));
    Serial.println(V_l, 2);
  }
}