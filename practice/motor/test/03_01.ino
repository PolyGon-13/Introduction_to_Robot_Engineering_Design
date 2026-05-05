#define pi 3.1416

// ============ PID ============
float kp = 0.0;
float ki = 0.0;
float kd = 0.0;

// ============ PIN ============
const byte interruptPinA = 3;
const byte interruptPinB = 5;
volatile long EncoderCount = 0;
const byte PWMPin = 6;
const byte DirPin1 = 7;
const byte DirPin2 = 8;

// ============ TIMING ============
unsigned long previousMillis = 0;
const unsigned long interval = 50;
volatile unsigned long count = 0;
unsigned long count_prev = 0;
unsigned long t;
unsigned long t_prev = 0;

// ============ CONTROL ============
float revolution, RPM, RPM_d = 0;
float revolution_prev = 0;
int dt;
float Vmax = 6;
float Vmin = -6;
float V = 0.0;
float e = 0, e_prev = 0, inte = 0, inte_prev = 0;

// ============ SERIAL ============
String inputString = "";
boolean inputComplete = false;
unsigned long inputTime = 0;
bool waiting = false;
int line_bool = 0;
int line_bbool = 0;
int line_time = 500;

// ============ 모드 ============
enum Mode { MODE_IDLE, MODE_TUNE, MODE_PID };
Mode ctrlMode = MODE_IDLE;

// ============ AUTO-TUNE ============
#define MAX_CROSSINGS 8   // 8번 교차 후 계산 (앞 4번은 안정화용)
float relay_amp = 4.0;    // 릴레이 출력 크기 (V), 진동 작으면 키우기
bool  tune_above = false;
int   crossing_count = 0;
unsigned long crossing_times[MAX_CROSSINGS];
float tune_rpm_max = -9999;
float tune_rpm_min =  9999;

// ============ 엔코더 ISR ============
void ISR_EncoderA() {
  bool PinB = digitalRead(interruptPinB);
  bool PinA = digitalRead(interruptPinA);
  if (PinB == LOW) EncoderCount += (PinA == HIGH) ? 1 : -1;
  else             EncoderCount += (PinA == HIGH) ? -1 : 1;
}
void ISR_EncoderB() {
  bool PinB = digitalRead(interruptPinA);
  bool PinA = digitalRead(interruptPinB);
  if (PinA == LOW) EncoderCount += (PinB == HIGH) ? -1 : 1;
  else             EncoderCount += (PinB == HIGH) ? 1 : -1;
}

void WriteDriverVoltage(float V, float Vmax) {
  int PWMval = constrain(int(255 * abs(V) / Vmax), 0, 255);
  if (V > 0) { digitalWrite(DirPin1, HIGH); digitalWrite(DirPin2, LOW); }
  else if (V < 0) { digitalWrite(DirPin1, LOW); digitalWrite(DirPin2, HIGH); }
  else { digitalWrite(DirPin1, LOW); digitalWrite(DirPin2, LOW); }
  analogWrite(PWMPin, PWMval);
}

void updateCount() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    count++;
  }
}

// ── AUTO-TUNE 시작 ──
void startAutoTune() {
  if (RPM_d <= 0) {
    Serial.println(">> 먼저 목표 RPM을 입력하세요!");
    return;
  }
  Serial.print(">> AUTO-TUNE 시작 | 목표 RPM: ");
  Serial.println(RPM_d);
  Serial.println(">> 모터가 진동합니다. 8번 교차 후 자동 완료...");
  ctrlMode = MODE_TUNE;
  tune_above = (RPM > RPM_d);
  crossing_count = 0;
  tune_rpm_max = -9999;
  tune_rpm_min =  9999;
  e_prev = 0; inte = 0; inte_prev = 0;
}

// ── AUTO-TUNE 완료 → kp/ki/kd 계산 ──
void finishAutoTune() {
  // crossing 4~7 기준으로 주기 계산 (앞 4개는 안정화)
  float Tu_ms = ((crossing_times[6] - crossing_times[4]) +
                 (crossing_times[7] - crossing_times[5])) / 2.0;
  float osc_amp = (tune_rpm_max - tune_rpm_min) / 2.0;

  if (osc_amp < 1.0) {
    Serial.println(">> 진동 너무 작음. relay_amp를 키우고 재시도하세요.");
    ctrlMode = MODE_IDLE;
    return;
  }

  // Ziegler-Nichols 공식
  float Ku = (4.0 * relay_amp) / (pi * osc_amp); // 단위: V/RPM
  kp = 0.6  * Ku;
  ki = kp   / (0.5   * Tu_ms);   // dt가 ms 단위이므로 Ti도 ms
  kd = kp   *  0.125 * Tu_ms;    // Td도 ms 단위

  Serial.println("====== AUTO-TUNE 완료 ======");
  Serial.print("Tu    = "); Serial.print(Tu_ms);   Serial.println(" ms");
  Serial.print("진폭  = "); Serial.println(osc_amp);
  Serial.print("Ku    = "); Serial.println(Ku, 4);
  Serial.print("kp    = "); Serial.println(kp, 4);
  Serial.print("ki    = "); Serial.println(ki, 6);
  Serial.print("kd    = "); Serial.println(kd, 4);
  Serial.println("==========================");
  Serial.println(">> PID 제어 시작!");

  e_prev = 0; inte = 0; inte_prev = 0;
  ctrlMode = MODE_PID;
}

void setup() {
  Serial.begin(9600);
  pinMode(interruptPinA, INPUT_PULLUP);
  pinMode(interruptPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinA), ISR_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinB), ISR_EncoderB, CHANGE);
  pinMode(DirPin1, OUTPUT);
  pinMode(DirPin2, OUTPUT);
  inputString.reserve(16);
  Serial.println(">> 준비완료. RPM 입력 후 'tune' 입력하세요.");
}

void loop() {
  // ── 시리얼 입력 ──
  if (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      inputComplete = true;
      inputTime = millis();
      waiting = true;
    } else {
      inputString += inChar;
    }
  }

  if (inputComplete) {
    inputString.trim();
    if (inputString == "tune") {
      startAutoTune();
    } else if (inputString == "stop") {
      ctrlMode = MODE_IDLE;
      WriteDriverVoltage(0, Vmax);
      Serial.println(">> 정지");
    } else {
      float val = inputString.toFloat();
      if (val > 0) {
        RPM_d = val;
        Serial.print(">> 목표 RPM 설정: "); Serial.println(RPM_d);
      }
      line_bbool = inputString.toInt();
    }
    inputString = "";
    inputComplete = false;
  }

  if (waiting && millis() - inputTime >= (unsigned long)line_time) {
    waiting = false;
    line_bool = line_bbool;
  }

  updateCount();

  if (count > count_prev) {
    t = millis();
    revolution = EncoderCount / 1012.0;
    dt = max((int)(t - t_prev), 1);   // 0 방지
    RPM = (revolution - revolution_prev) / (dt / 1000.0) * 60;
    e = RPM_d - RPM;

    // ── 모드별 제어 ──
    if (ctrlMode == MODE_TUNE) {
      bool now_above = (RPM > RPM_d);

      // 제로 크로싱 감지
      if (now_above != tune_above) {
        tune_above = now_above;
        if (crossing_count < MAX_CROSSINGS)
          crossing_times[crossing_count] = t;
        crossing_count++;
        Serial.print(">> 교차 감지 #"); Serial.println(crossing_count);

        // 안정화 이후(5번째~) 진폭 측정
        if (crossing_count >= MAX_CROSSINGS)
          finishAutoTune();
      }
      // 진폭 추적 (안정화 이후)
      if (crossing_count >= 4) {
        tune_rpm_max = max(tune_rpm_max, RPM);
        tune_rpm_min = min(tune_rpm_min, RPM);
      }
      // 릴레이 출력
      V = tune_above ? -relay_amp : relay_amp;
      WriteDriverVoltage(V, Vmax);

    } else if (ctrlMode == MODE_PID) {
      inte = inte_prev + ((float)dt * (e + e_prev) / 2.0);
      V = kp * e + ki * inte + (kd * (e - e_prev) / (float)dt);
      if (V > Vmax) { V = Vmax; inte = inte_prev; }
      if (V < Vmin) { V = Vmin; inte = inte_prev; }
      WriteDriverVoltage(V, Vmax);

      // 시리얼 플로터 출력 (PID 모드에서만)
      Serial.print(RPM_d);         Serial.print(",");
      Serial.print(RPM);           Serial.print(",");
      Serial.print(RPM_d * 1.1);   Serial.print(",");
      Serial.println(RPM_d * 0.9);

    } else {
      WriteDriverVoltage(0, Vmax);
    }

    revolution_prev = revolution;
    count_prev = count;
    t_prev = t;
    inte_prev = inte;
    e_prev = e;
  }
}
