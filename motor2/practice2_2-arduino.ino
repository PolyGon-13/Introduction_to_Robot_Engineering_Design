void setup(){
  Serial.begin(9600);//노트북과 통신
  Serial1.begin(9600);//라즈베리파이와 통신
}

void loop() {
  if (Serial1.available()) {
    String inputString = Serial1.readStringUntil('\n');
    inputString.trim();

    if (inputString.length() > 0) {
      int index = inputString.indexOf(' ');
      if (index != -1) {
        float v = inputString.substring(0, index).toFloat();
        float w = inputString.substring(index + 1).toFloat();

        Serial.print("v: ");
        Serial.print(v);
        Serial.print(" w: ");
        Serial.println(w);
      } else {
        Serial.println("[ERROR] 공백 구분자가 없습니다.");
      }
    }
  }
}


