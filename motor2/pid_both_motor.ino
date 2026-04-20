#define pi 3.1416

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
const float PPR = 1012.0;
float Vmax = 6.0, Vmin = -6.0;
unsigned long previousMillis = 0;
const unsigned long interval = 50;
volatile unsigned long count = 0;
unsigned long count_prev = 0;
unsigned long t = 0, t_prev = 0;
int dt = 0;

// ============ 시리얼 명령 ============
String inputString = "";
char currentAction = 'S';
int targetPWM = 0;

// ============ 엔코더 ISR ============
void ISR_Encoder_A_r() {
  bool pinB = digitalRead(ENC_B_r);
  bool pinA = digitalRead(ENC_A_r);
  if (pinB == LOW) EncoderCount_r += (pinA == HIGH) ?  1 : -1;
  else             EncoderCount_r += (pinA == HIGH) ? -1 :  1;
}

void ISR_Encoder_A_l() {
  bool pinB = digitalRead(ENC_B_l);
  bool pinA = digitalRead(ENC_A_l);
  if (pinB == LOW) EncoderCount_l += (pinA == HIGH) ?  1 : -1;
  else             EncoderCount_l += (pinA == HIGH) ? -1 :  1;
}

// ============ 모터 드라이버 출력 ============
void WriteDriverVoltage_r(float V) {
  int PWMval = constrain(int(255 * abs(V) / Vmax), 0, 255);
  if (V > 0) {
    digitalWrite(DirPin1_r, HIGH); digitalWrite(DirPin2_r, LOW);
  } else if (V < 0) {
    digitalWrite(DirPin1_r, LOW);  digitalWrite(DirPin2_r, HIGH);
  } else {
    digitalWrite(DirPin1_r, LOW);  digitalWrite(DirPin2_r, LOW);
  }
  analogWrite(PWMPin_r, PWMval);
}

void WriteDriverVoltage_l(float V) {
  int PWMval = constrain(int(255 * abs(V) / Vmax), 0, 255);
  if (V > 0) {
    digitalWrite(DirPin1_l, HIGH); digitalWrite(DirPin2_l, LOW);
  } else if (V < 0) {
    digitalWrite(DirPin1_l, LOW);  digitalWrite(DirPin2_l, HIGH);
  } else {
    digitalWrite(DirPin1_l, LOW);  digitalWrite(DirPin2_l, LOW);
  }
  analogWrite(PWMPin_l, PWMval);
}

// ============ 정지 ============
void stopMotors() {
  WriteDriverVoltage_r(0);
  WriteDriverVoltage_l(0);
  RPM_d_r = 0;
  RPM_d_l = 0;
  Serial.println("Stop");
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

  pinMode(ENC_A_r, INPUT_PULLUP);
  pinMode(ENC_B_r, INPUT_PULLUP);
  pinMode(ENC_A_l, INPUT_PULLUP);
  pinMode(ENC_B_l, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A_r), ISR_Encoder_A_r, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_l), ISR_Encoder_A_l, CHANGE);

  pinMode(PWMPin_r, OUTPUT); pinMode(DirPin1_r, OUTPUT); pinMode(DirPin2_r, OUTPUT);
  pinMode(PWMPin_l, OUTPUT); pinMode(DirPin1_l, OUTPUT); pinMode(DirPin2_l, OUTPUT);

  stopMotors();
  inputString.reserve(16);
}

// ============ loop ============
void loop() {
  // 시리얼 명령 수신
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    currentAction = command.charAt(0);
    targetPWM = (command.length() > 1) ? command.substring(1).toInt() : 0;
    targetPWM = constrain(targetPWM, 0, 255);

    float targetRPM = (targetPWM / 255.0) * 130.0;

    switch (currentAction) {
      case 'F':
        RPM_d_r = targetRPM;
        RPM_d_l = targetRPM;
        Serial.println("Forward");
        break;
      case 'B':
        RPM_d_r = -targetRPM;
        RPM_d_l = -targetRPM;
        Serial.println("Backward");
        break;
      case 'L':
        RPM_d_r =  targetRPM;
        RPM_d_l = -targetRPM;
        Serial.println("Left");
        break;
      case 'R':
        RPM_d_r = -targetRPM;
        RPM_d_l =  targetRPM;
        Serial.println("Right");
        break;
      case 'S':
        stopMotors();
        break;
    }
  }

  updateCount();

  // PID 루프 (50ms 주기)
  if (count > count_prev) {
    t = millis();
    dt = (t - t_prev);
    if (dt <= 0) dt = 1;

    // 우모터 RPM 계산
    revolution_r = EncoderCount_r / PPR;
    RPM_r = (revolution_r - revolution_r_prev) / (dt / 1000.0) * 60.0;

    // 좌모터 RPM 계산
    revolution_l = EncoderCount_l / PPR;
    RPM_l = (revolution_l - revolution_l_prev) / (dt / 1000.0) * 60.0;

    // 우모터 PID
    e_r = RPM_d_r - RPM_r;
    inte_r = inte_r_prev + (dt * (e_r + e_r_prev) / 2.0);
    V_r = kp_r * e_r + ki_r * inte_r + kd_r * (e_r - e_r_prev) / dt;
    if (V_r > Vmax) { V_r = Vmax; inte_r = inte_r_prev; }
    if (V_r < Vmin) { V_r = Vmin; inte_r = inte_r_prev; }

    // 좌모터 PID
    e_l = RPM_d_l - RPM_l;
    inte_l = inte_l_prev + (dt * (e_l + e_l_prev) / 2.0);
    V_l = kp_l * e_l + ki_l * inte_l + kd_l * (e_l - e_l_prev) / dt;
    if (V_l > Vmax) { V_l = Vmax; inte_l = inte_l_prev; }
    if (V_l < Vmin) { V_l = Vmin; inte_l = inte_l_prev; }

    WriteDriverVoltage_r(V_r);
    WriteDriverVoltage_l(V_l);

    // 시리얼 플로터 출력: RPM_r, RPM_l, 목표RPM
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
