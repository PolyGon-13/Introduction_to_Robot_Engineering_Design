#define ENC1_CHA 3
#define ENC1_CHB 5

volatile long e1cnt = 0;

void Enc1chA_ISR()
{
    if (digitalRead(ENC1_CHA) == HIGH)
    {
        if (digitalRead(ENC1_CHB) == LOW)
            e1cnt--;
        else
            e1cnt++;
    }
    else
    {
        if (digitalRead(ENC1_CHB) == HIGH)
            e1cnt--;
        else
            e1cnt++;
    }
}

void setup()
{
    Serial.begin(9600);

    pinMode(ENC1_CHA, INPUT_PULLUP);
    pinMode(ENC1_CHB, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC1_CHA), Enc1chA_ISR, CHANGE);

    Serial.println("Encoder count test");
    Serial.println("Send r to reset count");
}

void loop()
{
    if (Serial.available())
    {
        char c = Serial.read();
        if (c == 'r' || c == 'R')
        {
            noInterrupts();
            e1cnt = 0;
            interrupts();
            Serial.println("Reset count");
        }
    }

    noInterrupts();
    long count = e1cnt;
    interrupts();

    Serial.print("Count: ");
    Serial.print(count);
    Serial.print(" | abs: ");
    Serial.println(abs(count));

    delay(100);
}