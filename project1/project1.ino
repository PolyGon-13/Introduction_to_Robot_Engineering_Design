#include <Arduino.h>

#define PI_F 3.1416f

const byte PWMPin_r = 9; // 오른쪽 PWM 핀
const byte DirPin1_r = 10; // 오른쪽 방향 제어 핀 1
const byte DirPin2_r = 11; // 오른쪽 방향 제어 핀 2
const byte ENC_A_r = 2; // 오른쪽 엔코더 A 채널
const byte ENC_B_r = 4; // 오른쪽 엔코더 B 채널

const byte PWMPin_l = 6; // 왼쪽 PWM 핀
const byte DirPin1_l = 7; // 왼쪽 방향 제어 핀 1
const byte DirPin2_l = 8; // 왼쪽 방향 제어 핀 2
const byte ENC_A_l = 3; // 왼쪽 엔코더 A 채널
const byte ENC_B_l = 5; // 왼쪽 엔코더 B 채널

const bool INVERT_ENC_R = true; // 오른쪽 엔코더 방향 반전 여부
const bool INVERT_ENC_L = true; // 왼쪽 엔코더 방향 반전 여부

const bool REVERSE_MOTOR_R = true; // 오른쪽 모터 방향 반전 여부
const bool REVERSE_MOTOR_L = true; // 왼쪽 모터 방향 반전 여부

const float WHEEL_R = 0.034f; // 바퀴 반지름 (m)
//const float WHEEL_L = 0.2f; // 좌우 바퀴 사이 간격 (m)
const float PPR = 1012.0f; // 바퀴 1회전당 엔코더 카운트 수
const float COUNT_PER_M_CAL = 1.0f; // 주행 오차 보정 계수 (목표 엔코더 카운트를 몇 배로 늘릴지)
const float COUNT_PER_M = (PPR / (2.0f * PI_F * WHEEL_R)) * COUNT_PER_M_CAL; // 1m 당 엔코더 카운트 수
const long STOP_TOL_CNT = 35; // 목표 도달 허용 오차 (35카운트 이내로 가까워지면 정지시킴)

// 거리 이동 PID 튜닝값
float kp_pos = 0.1f;
float ki_pos = 0.002f;
float kd_pos = 0.000f;

// 좌우 동기화 PID 튜닝값
float kp_sync = 0.085f;
float ki_sync = 0.01f;
float kd_sync = 0.000f;

// 모터 전압 범위 (음수는 역방향)
const float V_MAX = 6.0f;
const float V_MIN = -6.0f;

// 출발/정지 가감속 설정
const unsigned long START_RAMP_MS = 900; // 출발시 천천시 가속 시간 (ms)
const float STOP_RAMP_M = 0.1f; // 목표 지점 근처에서 감속할 지점 (m)
const float MIN_RAMP_SCALE = 0.1f; // 가감속 시 최소 속도 비율 (%)

// 좌우 엔코더 누적 카운트
volatile long EncoderCount_r = 0;
volatile long EncoderCount_l = 0;

enum DriveState { 
  ST_IDLE, // 대기 상태
  ST_RUN, // 주행 중
  ST_DONE  // 목표 도달 완료
};
DriveState driveState = ST_IDLE;

// 주행 시작 시점의 좌우 엔코더 값
long startCount_r = 0;
long startCount_l = 0;

// 목표 카운트
long targetCount = 0;

// 전진이면 1, 후진이면 -1
float driveSign = 1.0f;

float e_pos_prev = 0; // 이전 위치 오차
float inte_pos = 0; // 위치 오차 누적값

float e_sync_prev = 0; // 이전 동기화 오차
float inte_sync = 0; // 동기화 오차 누적값

const unsigned long PID_INTERVAL_MS = 20; // PID 제어 주기 (ms)
unsigned long lastPidMs = 0; // 마지막 PID 제어 시점 (ms)
unsigned long driveStartMs = 0; // 주행 시작 시점 (ms)

String inputPi = ""; // 라즈베리파이에서 들어오는 문자열

// 오른쪽 엔코더 A 채널 카운트
void ISR_Encoder_A_r() {
  bool pinA = digitalRead(ENC_A_r);
  bool pinB = digitalRead(ENC_B_r);
  int delta;
  if (pinA == HIGH) delta = (pinB == LOW) ? -1 : 1;
  else delta = (pinB == HIGH) ? -1 : 1;
  if (INVERT_ENC_R) delta = -delta;
  EncoderCount_r += delta;
}
// 왼쪽 엔코더 A 채널 카운트
void ISR_Encoder_A_l() {
  bool pinA = digitalRead(ENC_A_l);
  bool pinB = digitalRead(ENC_B_l);
  int delta;
  if (pinA == HIGH) delta = (pinB == LOW) ? -1 : 1;
  else delta = (pinB == HIGH) ? -1 : 1;
  if (INVERT_ENC_L) delta = -delta;
  EncoderCount_l += delta;
}

// 오른쪽 모터 출력
static inline void writeDriver_r(float V) {
  V = constrain(V, V_MIN, V_MAX); // 목표 전압 제한
  int PWMval = constrain((int)(255.0f * fabs(V) / V_MAX), 0, 255); // 전압 크기를 PWM 값으로 변환
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
// 왼쪽 모터 출력
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

// 입력받은 거리만큼 직진/후진
void startStraightDrive(float distance_m) {
  if (fabs(distance_m) < 0.001f) {
    //Serial.println("[ERR] distance too small");
    return;
  }

  // 전진이면 1, 후진이면 -1
  driveSign = (distance_m >= 0.0f) ? 1.0f : -1.0f;

  // 엔코더 값 읽기
  noInterrupts();
  startCount_r = EncoderCount_r;
  startCount_l = EncoderCount_l;
  interrupts();

  // 목표 거리를 목표 카운트로 변환
  targetCount = (long)(fabs(distance_m) * COUNT_PER_M);

  // PID 제어 변수 초기화
  e_pos_prev = (float)targetCount; // 초기 위치 오차 (=목표 카운트)
  inte_pos = 0; // 위치 오차 누적값
  e_sync_prev = 0; // 동기화 오차 초기값
  inte_sync = 0; // 동기화 오차 누적값

  driveStartMs = millis(); // 주행 시작 시간
  lastPidMs = driveStartMs; // PID 제어 시점 초기화
  driveState = ST_RUN;

  //Serial.print("[START] d=");
  //Serial.print(distance_m, 4);
  //Serial.print(" m  target_cnt=");
  //Serial.println(targetCount);
}

// 전체 정지
void stopAll(const char *reason) {
  writeDriver_r(0);
  writeDriver_l(0);
  inte_pos = 0;
  inte_sync = 0;
  driveState = ST_IDLE;

  //Serial.print("[STOP] ");
  //Serial.println(reason);
}

// 명령 처리
void processCommand(String s) {
  s.trim(); // 앞뒤 공백 제거
  if (s.length() == 0) return;

  char c0 = s.charAt(0); // 첫 번째 문자

  if (c0 == 'D' || c0 == 'd') {
    float dist = s.substring(1).toFloat(); // 문자열을 실수로 변환
    if (dist == 0.0f) {
      //Serial.println("[ERR] zero distance");
      return;
    }
    startStraightDrive(dist);
  } else if (c0 == 'S' || c0 == 's') {
    stopAll("user");
  }
}

// 라즈베리파이에서 오는 Serail 값 읽기
void readSerial1Line() {
  while (Serial1.available()) {
    char c = (char)Serial1.read(); // 문자 하나 읽기

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