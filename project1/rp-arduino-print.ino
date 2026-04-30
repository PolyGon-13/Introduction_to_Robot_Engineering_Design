void setup()
{
    Serial.begin(9600);  // 노트북 시리얼 모니터
    Serial1.begin(9600); // 라즈베리파이
}

void loop()
{
    if (Serial1.available())
    {
        String inputString = Serial1.readStringUntil('\n');
        inputString.trim();

        if (inputString.length() > 0)
        {
            Serial.print("[RASPI] ");
            Serial.println(inputString);
        }
    }
}