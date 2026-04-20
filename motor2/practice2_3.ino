const byte PWMPin_r = 9, DirPin1_r = 10, DirPin2_r = 11;
const byte PWMPin_l = 6, DirPin1_l = 7, DirPin2_l = 8;
float wheel_R = 0.034;  // 바퀴 지름 3.4cm 
float wheel_l = 0.170;  // 바퀴 사이 거리 17cm
const int PWM_MAX = 255;
float right_rpm = 0;
float left_rpm = 0;

void setup() {
  Serial.begin(9600);      
  Serial1.begin(9600);     
  pinMode(DirPin1_r, OUTPUT);
  pinMode(DirPin2_r, OUTPUT);
  pinMode(PWMPin_l, OUTPUT);
  pinMode(DirPin1_l, OUTPUT);
  pinMode(DirPin2_l, OUTPUT);
  stopMotors();  // 모터 정지
}

void loop() {
  if (Serial1.available()) {  
    // 한 줄의 데이터를 모두 읽어들임 (개행문자까지)
    String inputString = Serial1.readStringUntil('\n');
    inputString.trim(); // 앞뒤 공백 제거

    if (inputString.length() > 0) {
      int index = inputString.indexOf(' ');
      if (index != -1) {
        // 공백을 기준으로 v와 w를 분리
        float v = inputString.substring(0, index).toFloat();
        float w = inputString.substring(index + 1).toFloat();

        float phi_l = //******수식 코드 작성******
        float phi_r = //******수식 코드 작성******
        
        int pwm_r = map(abs(phi_r), 0, 10.5, 0, PWM_MAX);
        int pwm_l = map(abs(phi_l), 0, 10.5, 0, PWM_MAX);
        setMotor(PWMPin_r, DirPin1_r, DirPin2_r, pwm_r, phi_r > 0);
        setMotor(PWMPin_l, DirPin1_l, DirPin2_l, pwm_l, phi_l > 0);
        Serial.print("Set V: ");
        Serial.print(v);
        Serial.print(", Set W: ");
        Serial.print(w);
        Serial.print("  Set pwm L: ");
        Serial.print(pwm_l);
        Serial.print(", Set pwm R: ");
        Serial.println(pwm_r);
      }
      else {
        Serial.println("입력 형식 오류: 공백이 없음");
      }
    }
  }
}

// 모터 제어 함수
void setMotor(byte pwmPin, byte dirPin1, byte dirPin2, int pwm, bool dir) {
  analogWrite(pwmPin, pwm);        // PWM 출력
  digitalWrite(dirPin1, dir);        // 방향 설정
  digitalWrite(dirPin2, !dir);       // 반대 방향 설정
}

// 모터 정지 함수
void stopMotors() {
  analogWrite(PWMPin_r, 0);
  digitalWrite(DirPin1_r, LOW);
  digitalWrite(DirPin2_r, LOW);
  analogWrite(PWMPin_l, 0);
  digitalWrite(DirPin1_l, LOW);
  digitalWrite(DirPin2_l, LOW);
}
