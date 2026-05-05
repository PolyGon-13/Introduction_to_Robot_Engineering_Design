const byte PWMPin_r = 9, DirPin1_r = 10, DirPin2_r = 11;
const byte PWMPin_l = 6, DirPin1_l = 7, DirPin2_l = 8;

void setup() {
    Serial.begin(9600);
    pinMode(PWMPin_r, OUTPUT);
    pinMode(DirPin1_r, OUTPUT);
    pinMode(DirPin2_r, OUTPUT);
    pinMode(PWMPin_l, OUTPUT);
    pinMode(DirPin1_l, OUTPUT);
    pinMode(DirPin2_l, OUTPUT);
    stopMotors();
}

void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        char action = command.charAt(0);
        int pwmValue = (command.length() > 1) ? command.substring(1).toInt() : 0;
        pwmValue = constrain(pwmValue, 0, 255);
        switch (action) {
            case 'F': moveForward(pwmValue); break;
            case 'B': moveBackward(pwmValue); break;
            case 'L': turnLeft(pwmValue); break;
            case 'R': turnRight(pwmValue); break;
            case 'S': stopMotors(); break;
        }
    }
}
//로봇 정지 함수
void stopMotors() {
    analogWrite(PWMPin_r, 0);
    analogWrite(PWMPin_l, 0);
    Serial.println("Stop");
}

//로봇 전진 함수
void moveForward(int pwm) {
    analogWrite(PWMPin_r, pwm);
    digitalWrite(DirPin1_r, HIGH);
    digitalWrite(DirPin2_r, LOW);
    analogWrite(PWMPin_l, pwm);
    digitalWrite(DirPin1_l, HIGH);
    digitalWrite(DirPin2_l, LOW);
    Serial.print(pwm);
    Serial.print(" ");
    Serial.println("Forward");
}

//로봇 후진 함수
void moveBackward(int pwm) {
    analogWrite(PWMPin_r, pwm);
    digitalWrite(DirPin1_r, /* 주석 삭제 후 수정 */ );
    digitalWrite(DirPin2_r, /* 주석 삭제 후 수정 */ );
    analogWrite(PWMPin_l, pwm);
    digitalWrite(DirPin1_l, /* 주석 삭제 후 수정 */ );
    digitalWrite(DirPin2_l, /* 주석 삭제 후 수정 */ );
    Serial.print(pwm);
    Serial.print(" ");
    Serial.println("Backward");
}

//로봇 좌회전 함수
void turnLeft(int pwm) {
    analogWrite(PWMPin_r, pwm);
    digitalWrite(DirPin1_r, /* 주석 삭제 후 수정 */ );
    digitalWrite(DirPin2_r, /* 주석 삭제 후 수정 */ );
    analogWrite(PWMPin_l, pwm);
    digitalWrite(DirPin1_l, /* 주석 삭제 후 수정 */ );
    digitalWrite(DirPin2_l, /* 주석 삭제 후 수정 */ );
    Serial.print(pwm);
    Serial.print(" ");
    Serial.println("Left");
}

//로봇 우회전 함수
void turnRight(int pwm) {
    analogWrite(PWMPin_r, pwm);
    digitalWrite(DirPin1_r, /* 주석 삭제 후 수정 */ );
    digitalWrite(DirPin2_r, /* 주석 삭제 후 수정 */ );
    analogWrite(PWMPin_l, pwm);
    digitalWrite(DirPin1_l, /* 주석 삭제 후 수정 */ );
    digitalWrite(DirPin2_l, /* 주석 삭제 후 수정 */ );
    Serial.print(pwm);
    Serial.print(" ");
    Serial.println("Right");
}

