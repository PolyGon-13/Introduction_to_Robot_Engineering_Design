#define ENC1_CHA 3
#define ENC1_CHB 5

// true로 바꾸면 count 증가/감소 방향이 반대로 바뀜
const bool INVERT_ENCODER_DIR = true;

volatile long e1cnt = 0;

void updateEncoderCount(int delta)
{
    if (INVERT_ENCODER_DIR)
        e1cnt -= delta;
    else
        e1cnt += delta;
}

void Enc1chA_ISR()
{
    if (digitalRead(ENC1_CHA) == HIGH)
    {
        if (digitalRead(ENC1_CHB) == LOW)
            updateEncoderCount(-1);
        else
            updateEncoderCount(1);
    }
    else
    {
        if (digitalRead(ENC1_CHB) == HIGH)
            updateEncoderCount(-1);
        else
            updateEncoderCount(1);
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