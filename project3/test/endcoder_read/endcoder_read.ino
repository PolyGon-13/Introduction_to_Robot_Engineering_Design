#include <Arduino.h>


const byte PWMPin_r = 9;
const byte DirPin1_r = 10;
const byte DirPin2_r = 11;
const byte ENC_A_r = 2;
const byte ENC_B_r = 4;

const byte PWMPin_l = 6;
const byte DirPin1_l = 7;
const byte DirPin2_l = 8;
const byte ENC_A_l = 3;
const byte ENC_B_l = 5;

const bool INVERT_ENC_R = true;
const bool INVERT_ENC_L = false;

const unsigned long BAUD_PI = 9600;
const unsigned long BAUD_USB = 115200;
const unsigned long SEND_INTERVAL_MS = 50;

volatile long encoder_count_r = 0;
volatile long encoder_count_l = 0;
unsigned long last_send_ms = 0;

void reset_encoder_counts() {
  noInterrupts();
  encoder_count_r = 0;
  encoder_count_l = 0;
  interrupts();
}

void isr_encoder_a_r() {
  bool a = digitalRead(ENC_A_r);
  bool b = digitalRead(ENC_B_r);
  int delta = (a == b) ? 1 : -1;
  if (INVERT_ENC_R) delta = -delta;
  encoder_count_r += delta;
}

void isr_encoder_a_l() {
  bool a = digitalRead(ENC_A_l);
  bool b = digitalRead(ENC_B_l);
  int delta = (a == b) ? 1 : -1;
  if (INVERT_ENC_L) delta = -delta;
  encoder_count_l += delta;
}

void stop_motor_outputs() {
  pinMode(PWMPin_r, OUTPUT);
  pinMode(DirPin1_r, OUTPUT);
  pinMode(DirPin2_r, OUTPUT);
  pinMode(PWMPin_l, OUTPUT);
  pinMode(DirPin1_l, OUTPUT);
  pinMode(DirPin2_l, OUTPUT);

  analogWrite(PWMPin_r, 0);
  digitalWrite(DirPin1_r, LOW);
  digitalWrite(DirPin2_r, LOW);
  analogWrite(PWMPin_l, 0);
  digitalWrite(DirPin1_l, LOW);
  digitalWrite(DirPin2_l, LOW);
}

void send_encoder_line(Stream &port, unsigned long now, long enc_l, long enc_r) {
  port.print(F("O,"));
  port.print(enc_l);
  port.print(',');
  port.print(enc_r);
  port.print(',');
  port.println(now);
}

void read_reset_commands(Stream &port) {
  while (port.available()) {
    char c = (char)port.read();
    if (c == 'R' || c == 'r') {
      reset_encoder_counts();
      port.println(F("RESET"));
      Serial.println(F("RESET"));
    }
  }
}

void setup() {
  stop_motor_outputs();

  pinMode(ENC_A_r, INPUT_PULLUP);
  pinMode(ENC_B_r, INPUT_PULLUP);
  pinMode(ENC_A_l, INPUT_PULLUP);
  pinMode(ENC_B_l, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A_r), isr_encoder_a_r, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_l), isr_encoder_a_l, CHANGE);

  Serial.begin(BAUD_USB);
  Serial1.begin(BAUD_PI);

  last_send_ms = millis();
  Serial.println(F("[READY] encoder read test"));
}

void loop() {
  read_reset_commands(Serial1);
  read_reset_commands(Serial);

  unsigned long now = millis();
  if (now - last_send_ms < SEND_INTERVAL_MS) {
    return;
  }
  last_send_ms = now;

  long enc_r;
  long enc_l;
  noInterrupts();
  enc_r = encoder_count_r;
  enc_l = encoder_count_l;
  interrupts();

  send_encoder_line(Serial1, now, enc_l, enc_r);
  send_encoder_line(Serial, now, enc_l, enc_r);
}
