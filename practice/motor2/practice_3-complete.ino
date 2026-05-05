const byte PWMPin_r = 9,  DirPin1_r = 10, DirPin2_r = 11;
const byte PWMPin_l = 6,  DirPin1_l = 7,  DirPin2_l = 8;

const float wheel_R  = 0.034;
const float wheel_l  = 0.170;
const int   PWM_MAX  = 255;
const float MAX_PHI  = 13.6;

const bool REVERSE_RIGHT = true;
const bool REVERSE_LEFT  = true;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  pinMode(PWMPin_r, OUTPUT);
  pinMode(DirPin1_r, OUTPUT);
  pinMode(DirPin2_r, OUTPUT);
  pinMode(PWMPin_l, OUTPUT);
  pinMode(DirPin1_l, OUTPUT);
  pinMode(DirPin2_l, OUTPUT);

  stopMotors();
  Serial.println("Ready. Input 'v w' from RaspberryPi.");
}

void loop() {
  if (Serial1.available()) {
    String inputString = Serial1.readStringUntil('\n');
    inputString.trim();

    if (inputString.length() == 0) return;

    int index = inputString.indexOf(' ');
    if (index == -1) {
      Serial.println("Input error: use 'v w' format. ex) 0.2 0");
      return;
    }

    float v = inputString.substring(0, index).toFloat();
    float w = inputString.substring(index + 1).toFloat();

    if (v == 0.0 && w == 0.0) {
      stopMotors();
      Serial.println("Stop");
      return;
    }

    float phi_l = (v / wheel_R) + ((wheel_l * w) / (2.0 * wheel_R));
    float phi_r = (v / wheel_R) - ((wheel_l * w) / (2.0 * wheel_R));

    int pwm_r = constrain((int)(abs(phi_r) / MAX_PHI * PWM_MAX), 0, PWM_MAX);
    int pwm_l = constrain((int)(abs(phi_l) / MAX_PHI * PWM_MAX), 0, PWM_MAX);

    bool dir_r = (phi_r > 0) ^ REVERSE_RIGHT;
    bool dir_l = (phi_l > 0) ^ REVERSE_LEFT;

    setMotor(PWMPin_r, DirPin1_r, DirPin2_r, pwm_r, dir_r);
    setMotor(PWMPin_l, DirPin1_l, DirPin2_l, pwm_l, dir_l);

    Serial.print("V: "); Serial.print(v);
    Serial.print(" | W: "); Serial.print(w);
    Serial.print(" | phi_l: "); Serial.print(phi_l);
    Serial.print(" | phi_r: "); Serial.print(phi_r);
    Serial.print(" | PWM_L: "); Serial.print(pwm_l);
    Serial.print(" | PWM_R: "); Serial.println(pwm_r);
  }
}

void setMotor(byte pwmPin, byte dirPin1, byte dirPin2, int pwm, bool dir) {
  analogWrite(pwmPin, pwm);
  digitalWrite(dirPin1, dir ? HIGH : LOW);
  digitalWrite(dirPin2, dir ? LOW  : HIGH);
}

void stopMotors() {
  analogWrite(PWMPin_r, 0);
  digitalWrite(DirPin1_r, LOW);
  digitalWrite(DirPin2_r, LOW);
  analogWrite(PWMPin_l, 0);
  digitalWrite(DirPin1_l, LOW);
  digitalWrite(DirPin2_l, LOW);
}
