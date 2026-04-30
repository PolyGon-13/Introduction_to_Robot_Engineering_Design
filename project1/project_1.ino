// ============================================================================
//  project_1.ino  -  정확한 직진 주행 (Hanyang Univ. ERICA, Robot Eng. Design)
// ----------------------------------------------------------------------------
//  하드웨어 : Arduino UNO R4 Minima + L298N + JGA25-370 x2 + 엔코더
//  통신     : Serial1(라즈베리파이, 9600bps) / Serial(노트북 디버그, 9600bps)
//
//  [rp-arduino_uart.py 명령 프로토콜]
//    숫자 입력  → "D <dist:.4f>\n"   ex) "D 1.5000"  → 전진 1.5 m
//               → "D -0.3000\n"              → 후진 0.3 m
//    s 입력    → "S\n"                        → 즉시 정지
//    q 입력    → "S\n" 송신 후 파이썬 종료
//
//  동작 : 사다리꼴 속도 프로파일 + 좌/우 모터 RPM PID + 동기화 PID
//         목표 거리 도달 시 자동 감속·정지 후 Serial1 으로 "DONE ..." 응답.
// ============================================================================
#include <Arduino.h>

#define PI_F 3.1416f

// ============================================================================
//  1) HARDWARE CONFIG
// ============================================================================
//  - 우(Right) 모터
const byte PWMPin_r  = 9;
const byte DirPin1_r = 10;
const byte DirPin2_r = 11;
const byte ENC_A_r   = 2;
const byte ENC_B_r   = 4;

//  - 좌(Left) 모터
const byte PWMPin_l  = 6;
const byte DirPin1_l = 7;
const byte DirPin2_l = 8;
const byte ENC_A_l   = 3;   // encoder_check.ino : ENC1_CHA = 3
const byte ENC_B_l   = 5;   // encoder_check.ino : ENC1_CHB = 5

//  ── 엔코더 방향 반전 ──────────────────────────────────────────────────────
//  encoder_check.ino 에서 INVERT_ENCODER_DIR = true 로 검증된 값.
//  전진 시 EncoderCount 가 증가해야 정상.
const bool INVERT_ENC_R = true;   // [TODO 검증] 우측은 아직 미검증
const bool INVERT_ENC_L = true;   // encoder_check.ino 로 확인 완료

//  ── 모터 드라이버 방향 반전 ───────────────────────────────────────────────
//  "D 0.1" 명령 후 실제 로봇이 전진하는지 확인. 한 쪽이 후진하면 해당 값 false.
const bool REVERSE_MOTOR_R = true;   // [TODO 확인] 실제 주행으로 검증
const bool REVERSE_MOTOR_L = true;   // [TODO 확인] 실제 주행으로 검증

// ============================================================================
//  2) ROBOT PARAMETERS  (★ 정확도에 가장 큰 영향 - 실측 후 보정 필수)
// ============================================================================
const float WHEEL_R   = 0.034f;    // 바퀴 반지름 [m]  ([TODO 측정] 실측값으로 교체)
const float WHEEL_L   = 0.170f;    // 휠 간격 [m]      (직진엔 영향 작음, [TODO 측정])
const float PPR       = 1012.0f;   // 1회전당 엔코더 펄스 (CHANGE 1ch 인터럽트 기준).
                                    // [TODO 검증] 1m 굴려보고 실제 카운트와 이론치 비교.

//  - 1 m 당 엔코더 카운트 (이론치). 보정 계수 CAL 로 미세 조정한다.
//  - 보정 절차: D 1.000 명령으로 주행 → 자(메저) 실측 거리 measured_m 측정 →
//                CAL_NEW = CAL_OLD * (1.000 / measured_m)
const float COUNT_PER_M_CAL  = 1.0000f;  // [TODO 캘리브레이션] 처음엔 1.0, 실측 후 갱신
const float COUNT_PER_M      = (PPR / (2.0f * PI_F * WHEEL_R)) * COUNT_PER_M_CAL;

// ============================================================================
//  3) MOTION PROFILE  (사다리꼴 + creep)
// ============================================================================
const float V_CRUISE    = 0.10f;   // 등속 구간 선속도 [m/s]   (첫 실차 주행용 저속)
const float ACCEL       = 0.18f;   // 가/감속도 [m/s^2]        (슬립/쏠림 방지용 완만 가속)
const float V_MIN_KICK  = 0.05f;   // 출발 직후 정지마찰 극복용 최소 속도 [m/s]
const float D_KICK      = 0.025f;  // 출발 후 KICK 유지 거리 [m]
const float D_CREEP     = 0.07f;   // 종료 직전 저속 접근(creep) 구간 거리 [m]
const float V_CREEP     = 0.035f;  // creep 속도 [m/s]
const long  STOP_TOL_CNT = 35;     // 정지 허용 카운트 오차 (좌우 평균 기준)

// ============================================================================
//  4) PID GAINS  (motor2/test/pid_both_motor.ino 의 값을 출발점으로 사용)
//     ※ 게인은 실험적으로 튜닝 필요 — 우선은 동일값으로 시작
// ============================================================================
//  - 우/좌 모터 RPM PID (단위 dt: ms — 기존 코드와 호환 유지)
float kp_r = 0.0800f, ki_r = 0.0008f, kd_r = 0.0800f;   // [TODO 튜닝] 첫 실행: 느린 우측을 조금 더 강하게
float kp_l = 0.0500f, ki_l = 0.0004f, kd_l = 0.0800f;   // [TODO 튜닝] 첫 실행: 빠른 좌측을 조금 더 부드럽게

//  - 동기화(Cross-coupling) PID : 좌우 엔코더 카운트 차이(=헤딩 오차)를 직접 0으로 만든다.
//    e_sync = (Δenc_r) - (Δenc_l)  [counts]
//    출력은 양 모터 V 에 ±로 더해진다 (전압 단위, V).
float kp_sync = 0.00120f;   // [TODO 튜닝] 첫 실행: 좌우 카운트 차이에 더 적극적으로 반응
float ki_sync = 0.00002f;   // [TODO 튜닝]
float kd_sync = 0.00020f;   // [TODO 튜닝] D항은 과한 순간 보정을 피하려고 낮춤
const float V_SYNC_LIMIT = 1.8f;  // 동기화 보정 전압의 최대 크기 [V]

// ============================================================================
//  5) LIMITS
// ============================================================================
const float V_MAX = 6.0f;   // L298N 출력 가용 전압 (PWM 정규화 기준)
const float V_MIN = -6.0f;

// ============================================================================
//  6) STATE
// ============================================================================
volatile long EncoderCount_r = 0;
volatile long EncoderCount_l = 0;

enum DriveState { ST_IDLE, ST_RUN, ST_BRAKE, ST_DONE };
DriveState driveState = ST_IDLE;

long  startCount_r = 0;
long  startCount_l = 0;
long  targetCount  = 0;       // 목표 절댓값 카운트 (좌우 평균 기준)
float driveSign    = 1.0f;    // +1: 전진, -1: 후진
unsigned long t_brakeStart = 0;

//  - PID 상태
float rev_r_prev = 0, rev_l_prev = 0;
float rpm_r = 0, rpm_l = 0;
float rpm_d = 0;              // 공통 목표 RPM (직진이므로 좌우 같음, 부호로 방향)

float e_r = 0, e_r_prev = 0, inte_r = 0;
float e_l = 0, e_l_prev = 0, inte_l = 0;
float V_r = 0, V_l = 0;

float e_sync_prev = 0, inte_sync = 0;
float V_sync = 0;

//  - 타이밍
const unsigned long PID_INTERVAL_MS = 20;  // 50 Hz
unsigned long lastPidMs = 0;

//  - 시리얼 입력 버퍼
String inputPi  = "";
String inputPc  = "";

// ============================================================================
//  7) ENCODER ISR  (ENC_A CHANGE 인터럽트 — encoder_check.ino 와 동일 로직)
// ============================================================================
//  기본 delta 계산은 encoder_check.ino 의 Enc1chA_ISR() 과 동일.
//  INVERT_ENC_* 플래그로 방향 반전 (encoder_check.ino: INVERT_ENCODER_DIR=true).
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

// ============================================================================
//  8) MOTOR DRIVER OUTPUT
// ============================================================================
static inline void writeDriver_r(float V) {
  V = constrain(V, V_MIN, V_MAX);
  int PWMval = (int)(255.0f * fabs(V) / V_MAX);
  PWMval = constrain(PWMval, 0, 255);
  if (V == 0) {
    digitalWrite(DirPin1_r, LOW); digitalWrite(DirPin2_r, LOW);
  } else {
    bool forward = (V > 0) ^ REVERSE_MOTOR_R;
    digitalWrite(DirPin1_r, forward ? HIGH : LOW);
    digitalWrite(DirPin2_r, forward ? LOW  : HIGH);
  }
  analogWrite(PWMPin_r, PWMval);
}
static inline void writeDriver_l(float V) {
  V = constrain(V, V_MIN, V_MAX);
  int PWMval = (int)(255.0f * fabs(V) / V_MAX);
  PWMval = constrain(PWMval, 0, 255);
  if (V == 0) {
    digitalWrite(DirPin1_l, LOW); digitalWrite(DirPin2_l, LOW);
  } else {
    bool forward = (V > 0) ^ REVERSE_MOTOR_L;
    digitalWrite(DirPin1_l, forward ? HIGH : LOW);
    digitalWrite(DirPin2_l, forward ? LOW  : HIGH);
  }
  analogWrite(PWMPin_l, PWMval);
}

// ============================================================================
//  9) UTILITIES
// ============================================================================
//  - 선속도 v(m/s) → 휠 RPM
static inline float vToRpm(float v) {
  // ω = v / R  [rad/s] →  RPM = ω * 60 / (2π)
  return (v / WHEEL_R) * 60.0f / (2.0f * PI_F);
}

//  - 거리(절댓값) 진행도(m)에 대한 사다리꼴(혹은 삼각) 프로파일 + creep
static float profileVelocity(float d_done, float d_total) {
  float d_left = d_total - d_done;
  if (d_left <= 0.0f) return 0.0f;

  //  종료 직전 creep : 정확도 향상 핵심
  if (d_left <= D_CREEP) {
    //  d_left가 0에 가까울수록 V_CREEP * (d_left/D_CREEP) 로 자연스럽게 감속
    float v = V_CREEP * (d_left / D_CREEP);
    if (v < V_CREEP * 0.35f) v = V_CREEP * 0.35f;  // 최저 속도 (정지마찰 회피)
    return v;
  }

  //  사다리꼴 가/감속 거리
  float d_acc = (V_CRUISE * V_CRUISE) / (2.0f * ACCEL);
  float d_dec = d_acc;
  float v;

  if (2.0f * d_acc >= d_total - D_CREEP) {
    //  너무 짧은 주행: 삼각 프로파일 (creep 영역 제외)
    float d_total_eff = d_total - D_CREEP;
    float half = d_total_eff * 0.5f;
    if (d_done < half)         v = sqrtf(2.0f * ACCEL * d_done);
    else                       v = sqrtf(2.0f * ACCEL * (d_total_eff - d_done));
  } else {
    if (d_done < d_acc)              v = sqrtf(2.0f * ACCEL * d_done);
    else if (d_left - D_CREEP < d_dec) v = sqrtf(2.0f * ACCEL * (d_left - D_CREEP));
    else                               v = V_CRUISE;
  }

  //  출발 KICK : 정지마찰 극복
  if (d_done < D_KICK && v < V_MIN_KICK) v = V_MIN_KICK;

  if (v > V_CRUISE) v = V_CRUISE;
  if (v < 0.0f)     v = 0.0f;
  return v;
}

// ============================================================================
//  10) COMMAND HANDLING
// ============================================================================
void startStraightDrive(float distance_m) {
  if (fabs(distance_m) < 0.001f) {
    Serial.println("[ERR] distance too small");
    return;
  }
  driveSign = (distance_m >= 0.0f) ? 1.0f : -1.0f;
  float d_abs = fabs(distance_m);

  noInterrupts();
  startCount_r = EncoderCount_r;
  startCount_l = EncoderCount_l;
  interrupts();

  targetCount = (long)(d_abs * COUNT_PER_M);

  //  PID 상태 초기화
  e_r = e_l = e_r_prev = e_l_prev = 0;
  inte_r = inte_l = 0;
  e_sync_prev = 0; inte_sync = 0; V_sync = 0;
  rpm_d = 0;
  rev_r_prev = (float)startCount_r / PPR;
  rev_l_prev = (float)startCount_l / PPR;
  lastPidMs  = millis();

  driveState = ST_RUN;

  Serial.print("[START] d="); Serial.print(distance_m, 4);
  Serial.print(" m  target_cnt="); Serial.println(targetCount);
}

void stopAll(const char* reason) {
  rpm_d = 0;
  V_r = V_l = 0;
  writeDriver_r(0);
  writeDriver_l(0);
  inte_r = inte_l = 0;
  inte_sync = 0;
  driveState = ST_IDLE;
  Serial.print("[STOP] "); Serial.println(reason);
}

void processCommand(String s) {
  s.trim();
  if (s.length() == 0) return;

  char c0 = s.charAt(0);
  if (c0 == 'D' || c0 == 'd') {
    String num = s.substring(1); num.trim();
    float dist = num.toFloat();
    if (dist == 0.0f) { Serial.println("[ERR] zero distance"); return; }
    startStraightDrive(dist);
  } else if (c0 == 'S' || c0 == 's') {
    stopAll("user");
    Serial1.println("STOPPED");
  } else {
    //  편의: 숫자만 들어오면 거리(m)로 해석
    float dist = s.toFloat();
    if (dist != 0.0f) startStraightDrive(dist);
    else Serial.println("[ERR] use 'D <m>' or 'S'");
  }
}

void readSerial1Line() {
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n' || c == '\r') {
      if (inputPi.length() > 0) { processCommand(inputPi); inputPi = ""; }
    } else if (inputPi.length() < 32) {
      inputPi += c;
    }
  }
}
void readSerialLine() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputPc.length() > 0) { processCommand(inputPc); inputPc = ""; }
    } else if (inputPc.length() < 32) {
      inputPc += c;
    }
  }
}

// ============================================================================
//  11) SETUP / LOOP
// ============================================================================
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  pinMode(ENC_A_r, INPUT_PULLUP); pinMode(ENC_B_r, INPUT_PULLUP);
  pinMode(ENC_A_l, INPUT_PULLUP); pinMode(ENC_B_l, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A_r), ISR_Encoder_A_r, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_l), ISR_Encoder_A_l, CHANGE);

  pinMode(PWMPin_r, OUTPUT); pinMode(DirPin1_r, OUTPUT); pinMode(DirPin2_r, OUTPUT);
  pinMode(PWMPin_l, OUTPUT); pinMode(DirPin1_l, OUTPUT); pinMode(DirPin2_l, OUTPUT);

  inputPi.reserve(40); inputPc.reserve(40);
  writeDriver_r(0); writeDriver_l(0);

  Serial.println(F("[READY] project_1: straight drive"));
  Serial.print(F("COUNT_PER_M = ")); Serial.println(COUNT_PER_M, 3);
  Serial.println(F("Cmd: 'D <m>' (e.g. D 1.5) or 'S'"));
}

void loop() {
  readSerial1Line();
  readSerialLine();

  unsigned long now = millis();
  if (now - lastPidMs < PID_INTERVAL_MS) return;
  unsigned long dt_ms = now - lastPidMs;
  if (dt_ms == 0) dt_ms = 1;
  lastPidMs = now;
  float dt_s = dt_ms / 1000.0f;

  //  엔코더 스냅샷
  long enc_r, enc_l;
  noInterrupts();
  enc_r = EncoderCount_r;
  enc_l = EncoderCount_l;
  interrupts();

  //  RPM 계산
  float rev_r = (float)enc_r / PPR;
  float rev_l = (float)enc_l / PPR;
  rpm_r = (rev_r - rev_r_prev) / dt_s * 60.0f;
  rpm_l = (rev_l - rev_l_prev) / dt_s * 60.0f;
  rev_r_prev = rev_r;
  rev_l_prev = rev_l;

  if (driveState == ST_IDLE || driveState == ST_DONE) {
    writeDriver_r(0); writeDriver_l(0);
    return;
  }

  //  좌우 평균 진행 카운트
  long progR = labs(enc_r - startCount_r);
  long progL = labs(enc_l - startCount_l);
  long progAvg = (progR + progL) / 2;

  if (driveState == ST_RUN) {
    if (progAvg >= targetCount - STOP_TOL_CNT) {
      driveState   = ST_BRAKE;
      t_brakeStart = now;
      rpm_d = 0;
    } else {
      float d_done  = (float)progAvg / COUNT_PER_M;
      float d_total = (float)targetCount / COUNT_PER_M;
      float v_d     = profileVelocity(d_done, d_total);
      rpm_d         = vToRpm(v_d) * driveSign;
    }
  }

  if (driveState == ST_BRAKE) {
    rpm_d = 0;
    bool stopped = (fabs(rpm_r) < 1.0f && fabs(rpm_l) < 1.0f);
    bool timeout = (now - t_brakeStart > 600);   // 600ms 이내 정지 안 되면 강제 종료
    if (stopped || timeout) {
      writeDriver_r(0); writeDriver_l(0);
      driveState = ST_DONE;

      long final_r = labs(EncoderCount_r - startCount_r);
      long final_l = labs(EncoderCount_l - startCount_l);
      float dist_r = (float)final_r / COUNT_PER_M;
      float dist_l = (float)final_l / COUNT_PER_M;
      float dist_avg = 0.5f * (dist_r + dist_l);

      //  결과 보고 (Serial1: 라즈베리파이, Serial: 디버그)
      char buf[96];
      snprintf(buf, sizeof(buf),
               "DONE %ld %ld %.4f %.4f %.4f",
               final_r, final_l, dist_r, dist_l, dist_avg);
      Serial1.println(buf);
      Serial.print(F("[DONE] enc_r=")); Serial.print(final_r);
      Serial.print(F(" enc_l=")); Serial.print(final_l);
      Serial.print(F(" d_r=")); Serial.print(dist_r, 4);
      Serial.print(F(" d_l=")); Serial.print(dist_l, 4);
      Serial.print(F(" d_avg=")); Serial.println(dist_avg, 4);
      return;
    }
  }

  //  ===== 동기화(Cross-coupling) PID =====
  //  좌우 진행 카운트 차이를 직접 0으로 수렴시킨다 → 직진성 향상
  long delta_enc = (enc_r - startCount_r) - (enc_l - startCount_l);
  //  후진 시 부호 보정 (sign 반영하여 항상 "전진 기준" 차이로 만든다)
  float e_sync = (float)delta_enc * driveSign;

  inte_sync += e_sync * dt_s;
  inte_sync  = constrain(inte_sync, -2000.0f, 2000.0f);
  float d_sync = (e_sync - e_sync_prev) / dt_s;
  V_sync = kp_sync * e_sync + ki_sync * inte_sync + kd_sync * d_sync;
  V_sync = constrain(V_sync, -V_SYNC_LIMIT, V_SYNC_LIMIT);
  e_sync_prev = e_sync;

  //  ===== 우(Right) 모터 RPM PID =====
  float rpm_d_r = rpm_d;
  e_r = rpm_d_r - rpm_r;
  inte_r += dt_ms * (e_r + e_r_prev) / 2.0f;
  inte_r  = constrain(inte_r, -1500.0f, 1500.0f);
  V_r = kp_r * e_r + ki_r * inte_r + kd_r * (e_r - e_r_prev) / dt_ms;
  //  r 가 더 멀리 갔을 때 e_sync>0 → 우모터 전압 -, 좌모터 전압 +
  V_r -= V_sync * driveSign;
  V_r  = constrain(V_r, V_MIN, V_MAX);
  e_r_prev = e_r;

  //  ===== 좌(Left) 모터 RPM PID =====
  float rpm_d_l = rpm_d;
  e_l = rpm_d_l - rpm_l;
  inte_l += dt_ms * (e_l + e_l_prev) / 2.0f;
  inte_l  = constrain(inte_l, -1500.0f, 1500.0f);
  V_l = kp_l * e_l + ki_l * inte_l + kd_l * (e_l - e_l_prev) / dt_ms;
  V_l += V_sync * driveSign;
  V_l  = constrain(V_l, V_MIN, V_MAX);
  e_l_prev = e_l;

  //  목표 RPM 0이고 실제 RPM도 거의 0이면 능동 0V (관성/PID 잔류 방지)
  if (driveState == ST_BRAKE && fabs(rpm_d) < 0.5f) {
    if (fabs(rpm_r) < 3.0f) V_r = 0;
    if (fabs(rpm_l) < 3.0f) V_l = 0;
  }

  writeDriver_r(V_r);
  writeDriver_l(V_l);

  //  ----- 디버그 출력 (200ms마다) -----
  static unsigned long lastLogMs = 0;
  if (now - lastLogMs > 200) {
    lastLogMs = now;
    Serial.print(F("st=")); Serial.print((int)driveState);
    Serial.print(F(" rpm_d=")); Serial.print(rpm_d, 1);
    Serial.print(F(" r=")); Serial.print(rpm_r, 1);
    Serial.print(F(" l=")); Serial.print(rpm_l, 1);
    Serial.print(F(" eR=")); Serial.print(enc_r - startCount_r);
    Serial.print(F(" eL=")); Serial.print(enc_l - startCount_l);
    Serial.print(F(" dEnc=")); Serial.print(delta_enc);
    Serial.print(F(" Vs=")); Serial.println(V_sync, 3);
  }
}
