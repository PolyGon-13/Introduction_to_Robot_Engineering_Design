#include <Arduino.h>

#define PI_F 3.1416f

// 핀 정의
const byte PWMPin_r = 9; // 오른쪽 모터 PWM 출력 핀
const byte DirPin1_r = 10; // 오른쪽 모터 방향 제어 핀 1
const byte DirPin2_r = 11; // 오른쪽 모터 방향 제어 핀 2
const byte ENC_A_r = 2; // 오른쪽 엔코더 A채널 (인터럽트)
const byte ENC_B_r = 4; // 오른쪽 엔코더 B채널 (방향 판단)

const byte PWMPin_l = 6; // 왼쪽 모터 PWM 출력 핀
const byte DirPin1_l = 7; // 왼쪽 모터 방향 제어 핀 1
const byte DirPin2_l = 8; // 왼쪽 모터 방향 제어 핀 2
const byte ENC_A_l = 3; // 왼쪽 엔코더 A채널
const byte ENC_B_l = 5; // 왼쪽 엔코더 B채널

// 방향 반전 설정
const bool INVERT_ENC_R = true; // 오른쪽 엔코더 방향 반전 여부
const bool INVERT_ENC_L = false; // 왼쪽 엔코더 방향 반전 여부
const bool REVERSE_MOTOR_R = true; // 오른쪽 모터 회전 방향 반전 여부
const bool REVERSE_MOTOR_L = true; // 왼쪽 모터 회전 방향 반전 여부

// 로봇 물리 파라미터
const float WHEEL_R = 0.034f; // 바퀴 반지름 (m)
const float WHEEL_BASE = 0.179f; // 좌우 바퀴 사이 간격 (m)
const float PPR = 1012.0f; // 바퀴 1회전당 엔코더 카운트 수
const float COUNT_PER_RAD = PPR / (2.0f * PI_F); // 1 라디안 회전당 엔코더 카운트 수

// 모터 드라이버 전압 제한
const float V_MAX = 6.0f; // 최대 출력 전압
const float V_MIN = -V_MAX; // 최소 출력 전압
const float DRIVER_DEADBAND_V = 0.05f; // 데드밴드 전압 (이 이하는 0으로 처리)

// PWM
int PWMval_R = 0;
int PWMval_L = 0;

// 속도 제한
const float WHEEL_SPEED_MAX = 8.0f; // 바퀴 최대 회전 속도 (rad/s)


// PID 제어 변수 구조체
struct WheelPID {
  float kp, ki, kd; // PID 게인
  float integ, prev_e; // 적분값과 이전 오차값
  float target; // 목표 각속도 (rad/s)
  float meas; // 측정된 각속도 (rad/s)
};
WheelPID pidR = {0.50f, 1.50f, 0.0f, 0, 0, 0, 0}; // 오른쪽 바퀴 PID 제어 변수 초기값
WheelPID pidL = {0.50f, 1.50f, 0.0f, 0, 0, 0, 0}; // 왼쪽 바퀴 PID 제어 변수 초기값

// Scale 및 Feedforward 상수
const float L_SCALE = 1.00f; // 왼쪽 PID 출력 스케일
const float R_SCALE = 1.00f; // 오른쪽 PID 출력 스케일
const float WHEEL_FF = 1.0f; // feedforward 계수
// WHEEL_FF : 목표 각속도에 비례해 출력되는 기본 전압 -> 너무 크면 빠르게 반응하지만 진동 가능성 증가, 너무 작으면 반응 느려짐


// 명령값
float V_cmd = 0.0f; // 현재 목표 선속도 (m/s)
float W_cmd = 0.0f; // 현재 목표 각속도 (rad/s)


// 타이머
const unsigned long PID_INTERVAL_MS = 20; // PID 제어 주기 (ms)
const unsigned long CMD_TIMEOUT_MS = 3000; // 명령 타임아웃 (ms) - 이 시간 동안 명령이 없으면 모션 정지
unsigned long lastPidMs = 0; // 마지막 PID 제어 시간
unsigned long lastCmdMs = 0; // 마지막 명령 수신 시간


// 엔코더 변수
volatile long EncoderCount_r = 0; // 오른쪽 엔코더 카운트
volatile long EncoderCount_l = 0; // 왼쪽 엔코더 카운트
long encR_prev = 0, encL_prev = 0; // 이전 엔코더 카운트


// 엔코더 인터럽트 함수
void ISR_Encoder_A_r() {
  bool pinA = digitalRead(ENC_A_r); // A채널 상태 읽기
  bool pinB = digitalRead(ENC_B_r); // B채널 상태 읽기
  int delta = (pinA == pinB) ? 1 : -1;
  if (INVERT_ENC_R) delta = -delta;
  EncoderCount_r += delta;
}
void ISR_Encoder_A_l() {
  bool pinA = digitalRead(ENC_A_l);
  bool pinB = digitalRead(ENC_B_l);
  int delta = (pinA == pinB) ? 1 : -1;
  if (INVERT_ENC_L) delta = -delta;
  EncoderCount_l += delta;
}


// 모터 드라이버 출력 함수
static inline void writeDriver_r(float V) {
  if (fabs(V) < DRIVER_DEADBAND_V) V = 0.0f; // 데드밴드 이하면 0
  V = constrain(V, V_MIN, V_MAX); // 전압 범위 제한
  PWMval_R = constrain((int)(255.0f * fabs(V) / V_MAX), 0, 255); // 전압 -> PWM 변환
  if (V == 0.0f) { // 정지 명령
    digitalWrite(DirPin1_r, LOW);
    digitalWrite(DirPin2_r, LOW);
  } else {
    bool forward = (V > 0) ^ REVERSE_MOTOR_R; // 전압 부호와 반전 설정으로 방향 결정
    // 방향 핀 설정
    digitalWrite(DirPin1_r, forward ? HIGH : LOW);
    digitalWrite(DirPin2_r, forward ? LOW : HIGH);
  }
  analogWrite(PWMPin_r, PWMval_R); // PWM 출력
}
static inline void writeDriver_l(float V) {
  if (fabs(V) < DRIVER_DEADBAND_V) V = 0.0f;
  V = constrain(V, V_MIN, V_MAX);
  PWMval_L = constrain((int)(255.0f * fabs(V) / V_MAX), 0, 255);
  if (V == 0.0f) {
    digitalWrite(DirPin1_l, LOW);
    digitalWrite(DirPin2_l, LOW);
  } else {
    bool forward = (V > 0) ^ REVERSE_MOTOR_L;
    digitalWrite(DirPin1_l, forward ? HIGH : LOW);
    digitalWrite(DirPin2_l, forward ? LOW : HIGH);
  }
  analogWrite(PWMPin_l, PWMval_L);
}

// PID 제어 변수 초기화
void resetWheelPID(WheelPID &p) {
  p.integ = 0.0f;
  p.prev_e = 0.0f;
  p.target = 0.0f;
}

// 정지 함수
void stopMotion() {
  V_cmd = 0.0f;
  W_cmd = 0.0f;
  resetWheelPID(pidR);
  resetWheelPID(pidL);
  writeDriver_r(0.0f);
  writeDriver_l(0.0f);
}

// PID 계산 함수
float computePID(WheelPID &p, float dt) {
  float e = p.target - p.meas; // 오차 
  p.integ += e * dt; // 오차 시간 적분
  p.integ = constrain(p.integ, -3.0f, 3.0f); // 적분 항 제한
  float d = (e - p.prev_e) / dt; // 오차 변화량
  p.prev_e = e; // 이전 오차 업데이트
  float V = WHEEL_FF * p.target + (p.kp * e + p.ki * p.integ + p.kd * d); // Feedforward + PID 계산
  return V;
}


// 속도 -> 바퀴 각속도 변환
// w > 0 : 좌회전, w < 0 : 우회전
void resolveWheelTargets(float v, float w) {
  float wL = (v - WHEEL_BASE * w * 0.5f) / WHEEL_R; // 왼쪽 바퀴 목표 각속도
  float wR = (v + WHEEL_BASE * w * 0.5f) / WHEEL_R; // 오른쪽 바퀴 목표 각속도
  pidL.target = constrain(wL, -WHEEL_SPEED_MAX, WHEEL_SPEED_MAX);
  pidR.target = constrain(wR, -WHEEL_SPEED_MAX, WHEEL_SPEED_MAX);
}


String inputPi  = "";

// 시리얼 명령 처리 함수
void processCommand(String s) {
  s.trim(); // 앞뒤 공백 제거
  if (s.length() == 0) return;
  char c0 = s.charAt(0); // 명령의 첫 글자 확인
  if (c0 == 'S' || c0 == 's') { // S 명령 : 정지
    stopMotion();
    lastCmdMs = millis();
  } 
  else if (c0 == 'V' || c0 == 'v') { // V 명령 : 속도 명령
    int comma = s.indexOf(','); // 쉼표 위치 찾기
    if (comma <= 1 || comma >= s.length() - 1) return; // 쉼표가 없거나 위치가 잘못된 경우
    V_cmd = s.substring(1, comma).toFloat(); // 선속도
    W_cmd = s.substring(comma + 1).toFloat(); // 각속도
    lastCmdMs = millis();
  }
}


// 라즈베리파이에서 오는 시리얼 명령 읽기
void readRPSerial() {
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n' || c == '\r') {
      if (inputPi.length() > 0) {
        processCommand(inputPi);
        inputPi = "";
      }
    } 
    else if (inputPi.length() < 32) {
      inputPi += c;
    }
  }
}


void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  // 엔코더 핀 설정
  pinMode(ENC_A_r, INPUT_PULLUP);
  pinMode(ENC_B_r, INPUT_PULLUP);
  pinMode(ENC_A_l, INPUT_PULLUP);
  pinMode(ENC_B_l, INPUT_PULLUP);

  // 엔코더 인터럽트 설정
  attachInterrupt(digitalPinToInterrupt(ENC_A_r), ISR_Encoder_A_r, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_l), ISR_Encoder_A_l, CHANGE);

  // 모터 드라이버 핀 설정
  pinMode(PWMPin_r, OUTPUT);
  pinMode(DirPin1_r, OUTPUT);
  pinMode(DirPin2_r, OUTPUT);
  pinMode(PWMPin_l, OUTPUT);
  pinMode(DirPin1_l, OUTPUT);
  pinMode(DirPin2_l, OUTPUT);

  inputPi.reserve(40); // 시리얼 입력 버퍼 미리 확보

  // 모터 정지
  writeDriver_r(0);
  writeDriver_l(0);

  // 시간 초기화
  lastPidMs = millis();
  lastCmdMs = millis();
  Serial.println(F("[READY] Robot Ready!"));
}


void loop() {
  readRPSerial(); // 라즈베리파이에서 오는 명령 읽기

  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) stopMotion(); // 마지막 명령 수신 후 일정 시간 지나면 정지

  // PID 제어 주기 제한
  unsigned long now = millis();
  if (now - lastPidMs < PID_INTERVAL_MS) return;
  float dt = (now - lastPidMs) / 1000.0f;
  if (dt < 0.001f) dt = 0.001f;
  lastPidMs = now;

  // 엔코더 카운트 읽기
  long enc_r, enc_l;
  noInterrupts();
  enc_r = EncoderCount_r;
  enc_l = EncoderCount_l;
  interrupts();

  // 엔코더 카운트 변화량 계산
  long dR = enc_r - encR_prev; encR_prev = enc_r;
  long dL = enc_l - encL_prev; encL_prev = enc_l;

  // 바퀴 각속도 계산 (rad/s)
  pidR.meas = (dR / COUNT_PER_RAD) / dt;
  pidL.meas = (dL / COUNT_PER_RAD) / dt;
  resolveWheelTargets(V_cmd, W_cmd);

  // 모터 출력
  float Vr = 0.0f;
  float Vl = 0.0f;
  if (V_cmd == 0.0f && W_cmd == 0.0f) {
    stopMotion();
  } 
  else {
    Vr = R_SCALE * computePID(pidR, dt);
    Vl = L_SCALE * computePID(pidL, dt);
    writeDriver_r(Vr);
    writeDriver_l(Vl);
  }
  /*
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
    Serial.print(F(" Vl=")); Serial.print(Vl, 2);
    Serial.print(F(" | PWM_R=")); Serial.print(PWMval_R);
    Serial.print(F(" PWM_L=")); Serial.println(PWMval_L);
  }
  */
}