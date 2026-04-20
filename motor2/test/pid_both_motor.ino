#define pi 3.1416

// ============ 방향 반전 플래그 ============
const bool REVERSE_RIGHT = true;
const bool REVERSE_LEFT  = true;

// ============ PID 파라미터 (우모터) ============
float kp_r = 0.06;
float ki_r = 0.0005;
float kd_r = 0.1;

// ============ PID 파라미터 (좌모터) ============
float kp_l = 0.06;
float ki_l = 0.0005;
float kd_l = 0.1;

// ============ 모터 핀 ============
const byte PWMPin_r = 9,  DirPin1_r = 10, DirPin2_r = 11;
const byte PWMPin_l = 6,  DirPin1_l = 7,  DirPin2_l = 8;

// ============ 엔코더 핀 ============
const byte ENC_A_r = 2, ENC_B_r = 4;
const byte ENC_A_l = 3, ENC_B_l = 5;

// ============ 엔코더 카운트 ============
volatile long EncoderCount_r = 0;
volatile long EncoderCount_l = 0;

// ============ 로봇 파라미터 ============
const float wheel_R = 0.034;   // 바퀴 반지름 (m)
const float wheel_L = 0.170;   // 휠 간격 (m)

// ============ PID 변수 (우모터) ============
float revolution_r = 0, revolution_r_prev = 0;
float RPM_r = 0, RPM_d_r = 0;
float e_r = 0, e_r_prev = 0;
float inte_r = 0, inte_r_prev = 0;
float V_r = 0;

// ============ PID 변수 (좌모터) ============
float revolution_l = 0, revolution_l_prev = 0;
float RPM_l = 0, RPM_d_l = 0;
float e_l = 0, e_l_prev = 0;
float inte_l = 0, inte_l_prev = 0;
float V_l = 0;

// ============ 타이밍 ============
const float PPR     = 1012.0;
const float Vmax    =  6.0;
const float Vmin    = -6.0;
const float RPM_MAX = 130.0;   // 모터 최대 RPM

unsigned long previousMillis = 0;
const unsigned long interval = 50;
volatile unsigned long count = 0;
unsigned long count_prev = 0;
unsigned long t = 0, t_prev = 0;
int dt = 0;

// ============ 엔코더 ISR ============
void ISR_Encoder_A_r() {
  bool pinB = digitalRead(ENC_B_r);
  bool pinA = digitalRead(ENC_A_r);
  if (REVERSE_RIGHT) {
    if (pinB == LOW) EncoderCount_r += (pinA == HIGH) ? -1 :  1;
    else             EncoderCount_r += (pinA == HIGH) ?  1 : -1;
  } else {
    if (pinB == LOW) EncoderCount_r += (pinA == HIGH) ?  1 : -1;
    else             EncoderCount_r += (pinA == HIGH) ? -1 :  1;
  }
}

void ISR_Encoder_A_l() {
  bool pinB = digitalRead(ENC_B_l);
  bool pinA = digitalRead(ENC_A_l);
  if (REVERSE_LEFT) {
    if (pinB == LOW) EncoderCount_l += (pinA == HIGH) ? -1 :  1;
    else             EncoderCount_l += (pinA == HIGH) ?  1 : -1;
  } else {
    if (pinB == LOW) EncoderCount_l += (pinA == HIGH) ?  1 : -1;
    else             EncoderCount_l += (pinA == HIGH) ? -1 :  1;
  }
}

// ============ 모터 드라이버 출력 ============
void WriteDriverVoltage_r(float V) {
  int PWMval = constrain(int(255 * abs(V) / Vmax), 0, 255);
  bool forward = (V > 0) ^ REVERSE_RIGHT;
  if (V == 0) {
    digitalWrite(DirPin1_r, LOW); digitalWrite(DirPin2_r, LOW);
  } else if (forward) {
    digitalWrite(DirPin1_r, HIGH); digitalWrite(DirPin2_r, LOW);
  } else {
    digitalWrite(DirPin1_r, LOW);  digitalWrite(DirPin2_r, HIGH);
  }
  analogWrite(PWMPin_r, PWMval);
}

void WriteDriverVoltage_l(float V) {
  int PWMval = constrain(int(255 * abs(V) / Vmax), 0, 255);
  bool forward = (V > 0) ^ REVERSE_LEFT;
  if (V == 0) {
    digitalWrite(DirPin1_l, LOW); digitalWrite(DirPin2_l, LOW);
  } else if (forward) {
    digitalWrite(DirPin1_l, HIGH); digitalWrite(DirPin2_l, LOW);
  } else {
    digitalWrite(DirPin1_l, LOW);  digitalWrite(DirPin2_l, HIGH);
  }
  analogWrite(PWMPin_l, PWMval);
}

// ============ 정지 ============
void stopMotors() {
  RPM_d_r = 0; RPM_d_l = 0;
  inte_r = 0;  inte_l = 0;
  inte_r_prev = 0; inte_l_prev = 0;
  e_r = 0; e_l = 0;
  e_r_prev = 0; e_l_prev = 0;
  WriteDriverVoltage_r(0);
  WriteDriverVoltage_l(0);
  Serial.println("Stop");
}

// ============ v,w → 좌우 목표 RPM 변환 ============
void setVW(float v, float w) {
  // phi (rad/s) 계산
  float phi_r = (v / wheel_R) - ((wheel_L * w) / (2.0 * wheel_R));
  float phi_l = (v / wheel_R) + ((wheel_L * w) / (2.0 * wheel_R));

  // rad/s → RPM
  RPM_d_r = phi_r * 60.0 / (2.0 * pi);
  RPM_d_l = phi_l * 60.0 / (2.0 * pi);

  // 적분 초기화 (목표 급변 시 windup 방지)
  inte_r = 0; inte_r_prev = 0;
  inte_l = 0; inte_l_prev = 0;
  e_r_prev = 0; e_l_prev = 0;

  Serial.print("V: "); Serial.print(v);
  Serial.print(" | W: "); Serial.print(w);
  Serial.print(" | RPM_d_r: "); Serial.print(RPM_d_r);
  Serial.print(" | RPM_d_l: "); Serial.println(RPM_d_l);
}

// ============ 타이머 카운트 ============
void updateCount() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    count++;
  }
}

// ============ setup ============
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  pinMode(ENC_A_r, INPUT_PULLUP); pinMode(ENC_B_r, INPUT_PULLUP);
  pinMode(ENC_A_l, INPUT_PULLUP); pinMode(ENC_B_l, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A_r), ISR_Encoder_A_r, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_l), ISR_Encoder_A_l, CHANGE);

  pinMode(PWMPin_r, OUTPUT); pinMode(DirPin1_r, OUTPUT); pinMode(DirPin2_r, OUTPUT);
  pinMode(PWMPin_l, OUTPUT); pinMode(DirPin1_l, OUTPUT); pinMode(DirPin2_l, OUTPUT);

  stopMotors();
  Serial.println("Ready. Input 'v w' from Serial1 (RaspberryPi).");
}

// ============ loop ============
void loop() {
  // Serial1: 라즈베리파이에서 "v w" 수신
  if (Serial1.available()) {
    String inputString = Serial1.readStringUntil('\n');
    inputString.trim();
    if (inputString.length() == 0) return;

    int idx = inputString.indexOf(' ');
    if (idx == -1) {
      Serial.println("[ERROR] use 'v w' format");
      return;
    }

    float v = inputString.substring(0, idx).toFloat();
    float w = inputString.substring(idx + 1).toFloat();

    if (v == 0.0 && w == 0.0) {
      stopMotors();
    } else {
      setVW(v, w);
    }
  }

  // Serial: PC 시리얼 모니터에서 직접 테스트 가능
  if (Serial.available()) {
    String inputString = Serial.readStringUntil('\n');
    inputString.trim();
    if (inputString.length() == 0) return;

    int idx = inputString.indexOf(' ');
    if (idx == -1) {
      Serial.println("[ERROR] use 'v w' format");
      return;
    }

    float v = inputString.substring(0, idx).toFloat();
    float w = inputString.substring(idx + 1).toFloat();

    if (v == 0.0 && w == 0.0) {
      stopMotors();
    } else {
      setVW(v, w);
    }
  }

  updateCount();

  // PID 루프 (50ms 주기)
  if (count > count_prev) {
    t = millis();
    dt = (int)(t - t_prev);
    if (dt <= 0) dt = 1;

    // RPM 계산
    revolution_r = EncoderCount_r / PPR;
    RPM_r = (revolution_r - revolution_r_prev) / (dt / 1000.0) * 60.0;

    revolution_l = EncoderCount_l / PPR;
    RPM_l = (revolution_l - revolution_l_prev) / (dt / 1000.0) * 60.0;

    // 우모터 PID
    e_r = RPM_d_r - RPM_r;
    inte_r = inte_r_prev + (dt * (e_r + e_r_prev) / 2.0);
    inte_r = constrain(inte_r, -1000.0, 1000.0);  // anti-windup
    V_r = kp_r * e_r + ki_r * inte_r + kd_r * (e_r - e_r_prev) / dt;
    V_r = constrain(V_r, Vmin, Vmax);

    // 좌모터 PID
    e_l = RPM_d_l - RPM_l;
    inte_l = inte_l_prev + (dt * (e_l + e_l_prev) / 2.0);
    inte_l = constrain(inte_l, -1000.0, 1000.0);  // anti-windup
    V_l = kp_l * e_l + ki_l * inte_l + kd_l * (e_l - e_l_prev) / dt;
    V_l = constrain(V_l, Vmin, Vmax);

    WriteDriverVoltage_r(V_r);
    WriteDriverVoltage_l(V_l);

    // 시리얼 플로터 출력
    Serial.print(RPM_r);
    Serial.print(",");
    Serial.print(RPM_l);
    Serial.print(",");
    Serial.println(RPM_d_r);

    // 상태 업데이트
    revolution_r_prev = revolution_r;
    revolution_l_prev = revolution_l;
    count_prev = count;
    t_prev = t;
    inte_r_prev = inte_r;
    inte_l_prev = inte_l;
    e_r_prev = e_r;
    e_l_prev = e_l;
  }
}
