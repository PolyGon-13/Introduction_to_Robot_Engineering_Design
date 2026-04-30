#define ENC1_CHA 3
#define ENC1_CHB 5

// 엔코더 카운트 방향을 뒤집을지 정하는 변수
// true면 방향 반전
const bool INVERT_ENCODER_DIR = true;

// 엔코더 카운트 저장 변수
volatile long e1cnt = 0;

// 엔코더 카운트 업데이트 함수
void updateEncoderCount(int delta)
{
    if (INVERT_ENCODER_DIR)
        e1cnt -= delta;
    else
        e1cnt += delta;
}

// 엔코더 A채널 상태가 바뀔 때마다 자동으로 실행되는 함수
void Enc1chA_ISR()
{
    if (digitalRead(ENC1_CHA) == HIGH) // A채널 HIGH
    {
        if (digitalRead(ENC1_CHB) == LOW) // B채널 LOW
            updateEncoderCount(-1);
        else
            updateEncoderCount(1);
    }
    else // A채널 LOW
    {
        if (digitalRead(ENC1_CHB) == HIGH) // B채널 HIGH
            updateEncoderCount(-1);
        else // B채널 LOW
            updateEncoderCount(1);
    }
}

void setup()
{
    Serial.begin(9600);

    pinMode(ENC1_CHA, INPUT_PULLUP);
    pinMode(ENC1_CHB, INPUT_PULLUP);

    // CHANGE : A채널 상태가 HIGH->LOW 또는 LOW->HIGH로 바뀌면 감지 -> 2체배
    attachInterrupt(digitalPinToInterrupt(ENC1_CHA), Enc1chA_ISR, CHANGE);

    Serial.println("Encoder count test");
}

void loop()
{
    // 엔코더 값 출력 리셋
    if (Serial.available())
    {
        char c = Serial.read();
        if (c == 'q' || c == 'Q')
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
    Serial.print(" | Abs: ");
    Serial.println(abs(count));

    delay(100);
}