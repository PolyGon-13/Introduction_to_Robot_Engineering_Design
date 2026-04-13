#define pi 3.1416

// ************ PID ************
float kp = 0.0; // 0.04
float ki = 0.000; // 0.003
float kd = 0.0; // 0.1

// *************PIN**************
const byte interruptPinA = 3;
const byte interruptPinB = 5;
volatile long EncoderCount = 0;
const byte PWMPin = 6;
const byte DirPin1 = 7;
const byte DirPin2 = 8;

// *************etc**************
unsigned long previousMillis = 0;
const unsigned long interval = 50;
volatile unsigned long count = 0;
unsigned long count_prev = 0;
unsigned long t;
unsigned long t_prev = 0;
float revolution, RPM, RPM_d = 0;
float revolution_prev = 0;
int dt;
float RPM_max = 130;
float Vmax = 6;
float Vmin = -6;
float V = 0.1;
float e, e_prev = 0, inte, inte_prev = 0;
int line = 0;
int line_bool = 0;
String inputString = "";
boolean inputComplete = false;
unsigned long inputTime = 0;
bool waiting = false;
int line_bbool=0;
int line_time = 500;
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

  inputString.reserve(16);
}

void loop() {
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
    RPM_d = inputString.toFloat();  // RPM 값 추출
    line_bbool = inputString.toInt(); // line 값을 정수로 변환하여 line_bool에 설정
    inputString = "";
    inputComplete = false;
  }

  if (waiting && millis() - inputTime >= line_time) {
    waiting = false;
    line_bool = line_bbool;  
  }

  updateCount();

  if (count > count_prev) {
    t = millis();
    revolution = EncoderCount / 1012.0;
    dt = (t - t_prev);
    RPM = (revolution - revolution_prev) / (dt / 1000.0) * 60;
    e = RPM_d - RPM;
    inte = inte_prev + (dt * (e + e_prev) / 2);
    V = kp * e + ki * inte + (kd * (e - e_prev) / dt);

    if (V > Vmax) {
      V = Vmax;
      inte = inte_prev;
    }
    if (V < Vmin) {
      V = Vmin;
      inte = inte_prev;
    }

    WriteDriverVoltage(V, Vmax);

    Serial.print(RPM);
    Serial.print(",");
    Serial.print(RPM_d);
    Serial.print(",");
    Serial.println(line_bool);

    revolution_prev = revolution;
    count_prev = count;
    t_prev = t;
    inte_prev = inte;
    e_prev = e;
  }
}

