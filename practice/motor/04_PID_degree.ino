#include <Arduino.h>
#define pi 3.1416

// ************ PID ************
float kp = 0.0;
float ki = 0.000 * 0.00;
float kd = 0.0;

// *************PIN**************
const byte interruptPinA = 2;
const byte interruptPinB = 3;
volatile long EncoderCount = 0;
const byte PWMPin = 6;
const byte DirPin1 = 5;
const byte DirPin2 = 4;

// *************etc**************
unsigned long previousMillis = 0;
const unsigned long interval = 1;  // 5ms마다 PID 갱신
volatile unsigned long count = 0;
unsigned long count_prev = 0;
unsigned long t;
unsigned long t_prev = 0;
float revolution, EncoderTarget = 0;  // 목표 엔코더 값
float revolution_prev = 0;
int dt;
float Vmax = 5;
float Vmin = -5;
float V = 0.1;
float e, e_prev = 0, inte, inte_prev = 0;

String inputString = "";
boolean inputComplete = false;

// *************encoder read**************
void ISR_EncoderA() {
  bool PinB = digitalRead(interruptPinB);
  bool PinA = digitalRead(interruptPinA);

  if (PinB == LOW) {
    EncoderCount += (PinA == HIGH) ? 1 : -1;
  } else {
    EncoderCount += (PinA == HIGH) ? -1 : 1;
  }
}

void ISR_EncoderB() {
  bool PinB = digitalRead(interruptPinA);
  bool PinA = digitalRead(interruptPinB);

  if (PinA == LOW) {
    EncoderCount += (PinB == HIGH) ? -1 : 1;
  } else {
    EncoderCount += (PinB == HIGH) ? 1 : -1;
  }
}

float sign(float x) {
  return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
}

// *** Motor Driver Functions *****
void WriteDriverVoltage(float V, float Vmax) {
  int PWMval = int(255 * abs(V) / Vmax);
  if (PWMval > 255) {
    PWMval = 255;
  }
  if (V > 0) {
    digitalWrite(DirPin1, HIGH);
    digitalWrite(DirPin2, LOW);
  } else if (V < 0) {
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, HIGH);
  } else {
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, LOW);
  }
  analogWrite(PWMPin, PWMval);
}

void updateCount() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    count++;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(interruptPinA, INPUT_PULLUP);
  pinMode(interruptPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinA), ISR_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinB), ISR_EncoderB, CHANGE);
  pinMode(DirPin1, OUTPUT);
  pinMode(DirPin2, OUTPUT);

  inputString.reserve(16);  // Reserve memory for input string
}

void loop() {
  // Serial 입력 처리
  if (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      inputComplete = true;
    } else {
      inputString += inChar;
    }
  }

  // 입력 값이 완성되면 목표 엔코더 값에 적용
  if (inputComplete) {
    EncoderTarget = inputString.toFloat();  // Convert input string to float
    inputString = "";                       // Clear the input string
    inputComplete = false;                  // Reset the flag
  }

  updateCount();  // Update count using millis()

  if (count > count_prev) {
    t = millis();
    dt = t - t_prev;  // dt 계산 (시간 차이)

    e = EncoderTarget - EncoderCount;            // 목표 엔코더 값과 현재 엔코더 값의 차이로 오차 계산
    inte = inte_prev + (dt * (e + e_prev) / 2);  // 적분 계산


    // PID 계산 후 V 값 처리
    V = kp * e + ki * inte + (kd * (e - e_prev) / dt);



    if (V > Vmax) {
      V = Vmax;
      inte = inte_prev;
    }
    if (V < Vmin) {

      V = Vmin;
      inte = inte_prev;
    }
    if (abs(V) < 0.02) {
      V = 0;
    } else if (abs(V) >= 0.02 && abs(V) < 1) {
      V = (V > 0) ? 0.85 : -0.85;
    }



    WriteDriverVoltage(V, Vmax);


    // 그래프 출력용 데이터 (숫자만 출력)
    // Serial Plotter에서 보기 위한 형식
    Serial.print("TargetEncoder:");
    Serial.print(EncoderTarget);
    Serial.print("\t");
    Serial.print("CurrentEncoder:");
    Serial.print(EncoderCount);
    Serial.print("V:");
    Serial.println(V);
    // 값 갱신
    revolution_prev = revolution;
    count_prev = count;
    t_prev = t;
    inte_prev = inte;
    e_prev = e;
  }
}
