#include <Arduino.h>

#define PI_F 3.1416f

const byte PWMPin_r = 9; // 오른쪽 모터 PWM 핀
const byte DirPin1_r = 10; // 오른쪽 모터 방향 제어 핀1
const byte DirPin2_r = 11; // 오른쪽 모터 방향 제어 핀2
const byte ENC_A_r = 2; // 오른쪽 엔코더 A 채널 (인터럽트 핀)
const byte ENC_B_r = 4; // 오른쪽 엔코더 B 채널

const byte PWMPin_l = 6; // 왼쪽 모터 PWM 핀
const byte DirPin1_l = 7; // 왼쪽 모터 방향 제어 핀1
const byte DirPin2_l = 8; // 왼쪽 모터 방향 제어 핀2
const byte ENC_A_l = 3; // 왼쪽 엔코더 A 채널 (인터럽트 핀)
const byte ENC_B_l = 5; // 왼쪽 엔코더 B 채널

// 엔코더 카운트 방향 반전 여부
const bool INVERT_ENC_R = true; // 오른쪽
const bool INVERT_ENC_L = true; // 왼쪽

// 모터 회전 방향 반전 여부
const bool REVERSE_MOTOR_R = true; // 오른쪽
const bool REVERSE_MOTOR_L = true; // 왼쪽

const float WHEEL_R = 0.034f; // 바퀴 반지름
const float WHEEL_L = 0.2f; // 좌우 바퀴 사이 거리
const float PPR = 1012.0f; // 1회전당 엔코더 펄스

const float COUNT_PER_M_CAL = 1.0204f;
const float COUNT_PER_M = (PPR / (2.0f * PI_F * WHEEL_R)) * COUNT_PER_M_CAL; // 1m당 엔코더 카운트 수
const long STOP_TOL_CNT = 35; // 정지 허용 카운트 오차

// 거리 이동 PID 튜닝
float kp_pos = 0.1;
float ki_pos = 0.0004f;
float kd_pos = 0.015f;

// 좌우 엔코더 차이 PID 튜닝
float kp_sync = 20.0f;
float ki_sync = 0.00f;
float kd_sync = 0.00f;
const float V_SYNC_LIMIT = 0.241f; // 동기화 보정 전압의 최대 크기 제한

// 모터 전압 범위
const float V_MAX = 6.0f;
const float V_MIN = -6.0f;

// ── 가감속 파라미터 ──────────────────────────────────────────────────────────
// 가속: 출발 시 전압 상한을 0에서 V_MAX까지 선형 증가
const float RAMP_UP_RATE   = 60.0f;  // V/s — V_MAX=6V 도달까지 약 100ms
// 감속: e_pos가 DECEL_ZONE_CNT 이하로 줄면 전압 상한을 선형 감소
const long  DECEL_ZONE_CNT = 250;    // 감속 시작 카운트 (≈5.3cm)
const float V_DECEL_MIN    = 1.5f;   // 정지 직전 최소 전압 상한
// ────────────────────────────────────────────────────────────────────────────

// 오른쪽,왼쪽 엔코더 카운트
volatile long EncoderCount_r = 0;
volatile long EncoderCount_l = 0;

// 주행 상태
enum DriveState
{
  ST_IDLE, // 대기 상태
  ST_RUN, // 주행 중
  ST_DONE // 목표 도달 완료
};
DriveState driveState = ST_IDLE; // 처음에는 대기 상태로 시작

// 주행 시작 순간 오른쪽, 왼쪽 엔코더 값
long startCount_r = 0;
long startCount_l = 0;

long targetCount = 0; // 목표 이동거리를 엔코더 카운트로 환산한 값
float driveSign = 1.0f; // +1: 전진, -1: 후진

float e_pos_prev = 0; // 이전 위치 오차
float inte_pos = 0; // 위치 오차 누적값

float e_sync_prev = 0; // 이전 좌우 차이 오차
float inte_sync = 0;   // 좌우 차이 오차 누적값

float V_ramp_limit = 0.0f; // 가속 램프 현재 전압 상한

const unsigned long PID_INTERVAL_MS = 20; // PID 계산 주기(50 Hz)
unsigned long lastPidMs = 0; // 마지막으로 PID 계산한 시간

String inputPi = ""; // 라즈베리파이에서 들어온 명령

// 오른쪽 엔코더 인터럽트 함수
void ISR_Encoder_A_r()
{
  bool pinA = digitalRead(ENC_A_r); // A 채널의 현재 상태
  bool pinB = digitalRead(ENC_B_r); // B 채널의 현재 상태
  int delta;
  if (pinA == HIGH)
    delta = (pinB == LOW) ? -1 : 1;
  else
    delta = (pinB == HIGH) ? -1 : 1;
  if (INVERT_ENC_R)
    delta = -delta;
  EncoderCount_r += delta; // 엔코더 카운트 업데이트
}

// 왼쪽 엔코더 인터럽트 함수
void ISR_Encoder_A_l()
{
  bool pinA = digitalRead(ENC_A_l);
  bool pinB = digitalRead(ENC_B_l);
  int delta;
  if (pinA == HIGH)
    delta = (pinB == LOW) ? -1 : 1;
  else
    delta = (pinB == HIGH) ? -1 : 1;
  if (INVERT_ENC_L)
    delta = -delta;
  EncoderCount_l += delta;
}

// 오른쪽 모터 제어
static inline void writeDriver_r(float V)
{
  V = constrain(V, V_MIN, V_MAX);
  // 전압 크기를 PWM 값으로 변환 (0~255)
  int PWMval = constrain((int)(255.0f * fabs(V) / V_MAX), 0, 255);

  // 전압이 0이면 정지
  if (V == 0.0f)
  {
    digitalWrite(DirPin1_r, LOW);
    digitalWrite(DirPin2_r, LOW);
  }
  else
  {
    bool forward = (V > 0) ^ REVERSE_MOTOR_R;
    digitalWrite(DirPin1_r, forward ? HIGH : LOW);
    digitalWrite(DirPin2_r, forward ? LOW : HIGH);
  }
  analogWrite(PWMPin_r, PWMval);
}

// 왼쪽 모터 제어
static inline void writeDriver_l(float V)
{
  V = constrain(V, V_MIN, V_MAX);
  int PWMval = constrain((int)(255.0f * fabs(V) / V_MAX), 0, 255);
  if (V == 0.0f)
  {
    digitalWrite(DirPin1_l, LOW);
    digitalWrite(DirPin2_l, LOW);
  }
  else
  {
    bool forward = (V > 0) ^ REVERSE_MOTOR_L;
    digitalWrite(DirPin1_l, forward ? HIGH : LOW);
    digitalWrite(DirPin2_l, forward ? LOW : HIGH);
  }
  analogWrite(PWMPin_l, PWMval);
}

// 직진 주행 시작
void startStraightDrive(float distance_m)
{
  if (fabs(distance_m) < 0.001f)
  {
    Serial.println("[ERR] distance too small");
    return;
  }
  // 입력 거리가 양수면 전진, 음수면 후진
  driveSign = (distance_m >= 0.0f) ? 1.0f : -1.0f;

  noInterrupts();
  startCount_r = EncoderCount_r;
  startCount_l = EncoderCount_l;
  interrupts();

  // 입력 거리를 엔코더 카운트로 환산
  targetCount = (long)(fabs(distance_m) * COUNT_PER_M);

  e_pos_prev = (float)targetCount; // 초기 위치 오차 설정 (목표 카운트에서 시작)

  // PID 제어 변수 초기화
  inte_pos = 0;
  e_sync_prev = 0;
  inte_sync = 0;

  V_ramp_limit = 0.0f; // 가속 램프 초기화

  lastPidMs = millis(); // PID 시작 시간 초기화
  driveState = ST_RUN;  // 주행 상태를 RUN으로 설정

  // 입력 거리 출력
  Serial.print("[START] d=");
  Serial.print(distance_m, 4);
  // 목표 카운트 출력
  Serial.print(" m  target_cnt=");
  Serial.println(targetCount);
}

// 로봇 정지 및 상태 초기화
void stopAll(const char *reason)
{
  // 오른쪽,왼쪽 모터 모두 정지
  writeDriver_r(0);
  writeDriver_l(0);

  // PID 제어 변수 초기화
  inte_pos = 0;
  inte_sync = 0;
  V_ramp_limit = 0.0f;

  driveState = ST_IDLE; // 대기 상태로 전환

  // 정지 이유 출력
  Serial.print("[STOP] ");
  Serial.println(reason);
}

// 라즈베리파이나 PC에서 받은 문자열 명령 처리
void processCommand(String s)
{
  s.trim(); // 앞뒤 공백, 줄바꿈 제거
  if (s.length() == 0)
    return;

  char c0 = s.charAt(0);
  if (c0 == 'D' || c0 == 'd')
  {
    float dist = s.substring(1).toFloat();
    if (dist == 0.0f)
    {
      Serial.println("[ERR] zero distance");
      return;
    }
    startStraightDrive(dist); // 주행 시작
  }
  else if (c0 == 'S' || c0 == 's')
  {
    stopAll("user");
    //Serial1.println("STOPPED"); // 라즈베리파이에 정지 메시지 전송
  }
  else
  {
    Serial.println("[ERR] use 'D <m>' or 'S'");
  }
}

// 시리얼1에서 한 줄씩 읽어서 명령 처리
void readSerial1Line()
{
  while (Serial1.available())
  {
    char c = (char)Serial1.read();
    if (c == '\n' || c == '\r')
    {
      if (inputPi.length() > 0)
      {
        processCommand(inputPi);
        inputPi = "";
      }
    }
    else if (inputPi.length() < 32)
      inputPi += c;
  }
}
void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600); // 라즈베리파이와의 시리얼 통신 시작

  pinMode(ENC_A_r, INPUT_PULLUP); // 오른쪽 엔코더 A 채널
  pinMode(ENC_B_r, INPUT_PULLUP); // 오른쪽 엔코더 B 채널

  pinMode(ENC_A_l, INPUT_PULLUP); // 왼쪽 엔코더 A 채널
  pinMode(ENC_B_l, INPUT_PULLUP); // 왼쪽 엔코더 B 채널

  attachInterrupt(digitalPinToInterrupt(ENC_A_r), ISR_Encoder_A_r, CHANGE); // 오른쪽 엔코더 A 채널 인터럽트 설정
  attachInterrupt(digitalPinToInterrupt(ENC_A_l), ISR_Encoder_A_l, CHANGE); // 왼쪽 엔코더 A 채널 인터럽트 설정

  pinMode(PWMPin_r, OUTPUT);  // 오른쪽 모터 PWM 핀
  pinMode(DirPin1_r, OUTPUT); // 오른쪽 모터 방향 제어 핀1
  pinMode(DirPin2_r, OUTPUT); // 오른쪽 모터 방향 제어 핀2

  pinMode(PWMPin_l, OUTPUT);  // 왼쪽 모터 PWM 핀
  pinMode(DirPin1_l, OUTPUT); // 왼쪽 모터 방향 제어 핀1
  pinMode(DirPin2_l, OUTPUT); // 왼쪽 모터 방향 제어 핀2

  inputPi.reserve(40);

  writeDriver_r(0); // 오른쪽 모터 정지
  writeDriver_l(0); // 왼쪽 모터 정지

  Serial.println(F("[READY] Setup Complete!"));
}

void loop()
{
  readSerial1Line(); // 라즈베리파이에서 명령 읽기

  unsigned long now = millis(); // 현재 시간 (ms)
  if (now - lastPidMs < PID_INTERVAL_MS)
    return;
  float dt_s = (now - lastPidMs) / 1000.0f; // 지난 PID 계산
  if (dt_s < 0.001f)
    dt_s = 0.001f;
  lastPidMs = now; // 이번 PID 계산 시간 업데이트

  long enc_r, enc_l; // 오른쪽, 왼쪽 엔코더 카운트 읽기
  noInterrupts();
  enc_r = EncoderCount_r;
  enc_l = EncoderCount_l;
  interrupts();

  // 대기 상태이거나 도착 완료 상태이면 정지
  if (driveState == ST_IDLE || driveState == ST_DONE)
  {
    writeDriver_r(0);
    writeDriver_l(0);
    return;
  }

  // 양쪽 엔코더의 이동거리 평균 계산
  long progR = labs(enc_r - startCount_r);
  long progL = labs(enc_l - startCount_l);
  long progAvg = (progR + progL) / 2;

  // 남은 거리 오차
  float e_pos = (float)(targetCount - progAvg);

  // 남은 오차가 허용 범위 이하인 경우
  if (e_pos <= (float)STOP_TOL_CNT)
  {
    // 정지
    writeDriver_r(0);
    writeDriver_l(0);
    driveState = ST_DONE;

    // 라즈베리파이에 보낼 문자열 생성
    float dist_avg = 0.5f * (progR + progL) / COUNT_PER_M; // 좌우 평균 이동 카운트를 거리로 환산
    char buf[64];
    snprintf(buf, sizeof(buf), "DONE %ld %ld %.4f", progR, progL, dist_avg);
    //Serial1.println(buf); // 라즈베리파이에 메시지 전송
    return;
  }

  // 위치 PID 제어
  inte_pos += e_pos * dt_s; // 위치 오차 누적값 업데이트
  inte_pos = constrain(inte_pos, -20000.0f, 20000.0f); // 누적오차 제한
  float d_pos = (e_pos - e_pos_prev) / dt_s; // 위치 오차 변화률
  float V_base = kp_pos * e_pos + ki_pos * inte_pos + kd_pos * d_pos; // 위치 PID 제어로 기본 모터 전압 계산
  V_base = constrain(V_base * driveSign, V_MIN, V_MAX); // 모터 전압 범위로 제한
  e_pos_prev = e_pos; // 위치 오차 저장

  // ── 가감속 전압 상한 계산 ────────────────────────────────────────────────
  // 가속: 매 PID 주기마다 RAMP_UP_RATE * dt_s 씩 상한 증가 → 약 100ms에 V_MAX 도달
  V_ramp_limit += RAMP_UP_RATE * dt_s;
  if (V_ramp_limit > V_MAX) V_ramp_limit = V_MAX;

  // 감속: 목표까지 남은 카운트가 DECEL_ZONE_CNT 이하이면 전압 상한 선형 감소
  float V_limit = V_ramp_limit;
  if (e_pos < (float)DECEL_ZONE_CNT)
  {
    // DECEL_ZONE_CNT → V_MAX,  STOP_TOL_CNT → V_DECEL_MIN 으로 선형 보간
    float t = (e_pos - (float)STOP_TOL_CNT) / (float)(DECEL_ZONE_CNT - STOP_TOL_CNT);
    t = constrain(t, 0.0f, 1.0f);
    float V_decel_cap = V_DECEL_MIN + (V_MAX - V_DECEL_MIN) * t;
    if (V_decel_cap < V_limit) V_limit = V_decel_cap;
  }

  // V_base에 전압 상한 적용 (부호 유지)
  V_base = constrain(V_base, -V_limit, V_limit);
  // ────────────────────────────────────────────────────────────────────────

  // 좌우 엔코더 차이 PID 제어
  long delta_enc = (enc_r - startCount_r) - (enc_l - startCount_l); // 오른쪽과 왼쪽 엔코더의 이동거리 차이 계산
  float e_sync = (float)delta_enc * driveSign;
  inte_sync += e_sync * dt_s; // 좌우 차이 오차 누적값 업데이트
  inte_sync = constrain(inte_sync, -2000.0f, 2000.0f); // 누적오차 제한
  float d_sync = (e_sync - e_sync_prev) / dt_s; // 좌우 차이 오차 변화률
  float V_sync = constrain(kp_sync * e_sync + ki_sync * inte_sync + kd_sync * d_sync, -V_SYNC_LIMIT, V_SYNC_LIMIT); // 좌우 모터에 줄 보정 전압 계산 및 제한
  e_sync_prev = e_sync; // 좌우 차이 오차 저장

  writeDriver_r(constrain(V_base - V_sync * driveSign, V_MIN, V_MAX)); // 오른쪽 모터 최종 전압 명령
  writeDriver_l(constrain(V_base + V_sync * driveSign, V_MIN, V_MAX)); // 왼쪽 모터 최종 전압 명령

  // 디버그 출력
  static unsigned long lastLogMs = 0;
  if (now - lastLogMs > 200)
  {
    lastLogMs = now;
    Serial.print(F("prog="));
    Serial.print(progAvg);
    Serial.print(F(" e_pos="));
    Serial.print(e_pos, 0);
    Serial.print(F(" Vb="));
    Serial.print(V_base, 2);
    Serial.print(F(" Vlim="));
    Serial.print(V_limit, 2);
    Serial.print(F(" dEnc="));
    Serial.print(delta_enc);
    Serial.print(F(" Vs="));
    Serial.println(V_sync, 3);
    Serial.print(F(" R="));
    Serial.print(progR);
    Serial.print(F(" L="));
    Serial.print(progL);
  }
}
