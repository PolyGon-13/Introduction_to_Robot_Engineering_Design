// Motor 1 Encoder Pins
#define ENC1_CHA 3  // Motor1 Encoder CHA
#define ENC1_CHB 5  // Motor1 Encoder CHB

// Encoder counter variable
volatile long e1cnt = 0;  // Motor1 encoder count

// 인터럽트 서비스 루틴 (모터 1)
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
  attachInterrupt(digitalPinToInterrupt(ENC1_CHA), Enc1chA_ISR, CHANGE);
}

void loop() {
  Serial.print("Motor1 Encoder Count: ");
  Serial.println(e1cnt);
  delay(100);
}
