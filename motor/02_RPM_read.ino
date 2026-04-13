//RPM이 PWM 255 인가 시 130 RPM이 맞으나 L298N에서 전압강하로 인해 10V정도 나옴
// 12:130=10:X 를 풀어보면 108정도의 RPM이 최대값이 나옴
// 12는 뭐냐? 12V = 풀 전압 = PWM 255 = RPM 130 이라는 느낌

#define ENC1_CHA 3
#define ENC1_CHB 5
#define M1_PWM 6
#define M1_DIR1 7
#define M1_DIR2 8
#define PULSES_PER_ROTATION 1012

volatile long e1cnt = 0, lastE1Cnt = 0;
float rpm1 = 0.0;
int pwmValue = 0;
unsigned long lastMillis1 = 0;

void Enc1chA_ISR() {
  if (digitalRead(ENC1_CHA) == HIGH) {
    if (digitalRead(ENC1_CHB) == LOW) e1cnt--;
    else e1cnt++;
  } else {
    if (digitalRead(ENC1_CHB) == HIGH) e1cnt--;
    else e1cnt++;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(ENC1_CHA, INPUT);
  pinMode(ENC1_CHB, INPUT);
  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_DIR1, OUTPUT);
  pinMode(M1_DIR2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENC1_CHA), Enc1chA_ISR, CHANGE);
}

void loop() {
  unsigned long currentMillis = millis();

  // 1초마다 RPM 계산
  if (currentMillis - lastMillis1 >= 1000) {
    float pulses = e1cnt - lastE1Cnt;
    rpm1 = pulses * 60.0 / PULSES_PER_ROTATION; // rpm 코드 작성
    lastE1Cnt = e1cnt;
    lastMillis1 = currentMillis;
    Serial.print("Motor 1 RPM: ");
    Serial.println(rpm1);
  }
  // 시리얼 입력 처리 (PWM 값 입력)
  if (Serial.available()) {
    int newPwm = Serial.parseInt();
    if (newPwm != 0 || Serial.peek() == '\n') {
      pwmValue = newPwm;
      controlMotor1(pwmValue);
    }
  }
}

// 모터 제어 함수
void controlMotor1(int pwm) {
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    analogWrite(M1_PWM, pwm);
    digitalWrite(M1_DIR1, HIGH);
    digitalWrite(M1_DIR2, LOW);
  } else if (pwm < 0) {
    analogWrite(M1_PWM, -pwm);
    digitalWrite(M1_DIR1, LOW);
    digitalWrite(M1_DIR2, HIGH);
  } else {
    analogWrite(M1_PWM, 0);
    digitalWrite(M1_DIR1, LOW);
    digitalWrite(M1_DIR2, LOW);
  }
}
