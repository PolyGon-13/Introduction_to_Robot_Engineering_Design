import sys
import time
import serial

DEFAULT_PORT = "/dev/ttyS0" # 사용할 시리얼 포트 경로
DEFAULT_BAUD = 9600 # 사용할 시리얼 포트 속도

def open_serial():
    ser = serial.Serial(DEFAULT_PORT, DEFAULT_BAUD, timeout=0.1)
    time.sleep(2.0)
    ser.reset_input_buffer() # 아두이노가 재시작하면서 보낼 수 있는 초기 메시지를 버퍼에서 제거
    return ser

# 아두이노로 문자열 보냄
def send_line(ser, line):
    # 보낼 문자열 끝에 줄바꿈 문자 없으면 추가
    if not line.endswith("\n"):
        line += "\n"
    ser.write(line.encode("ascii"))
    ser.flush()

# 사용자 입력 받음
def interactive_loop(ser):
    while True:
        cmd = input(">> ").strip()

        # 엔터만 쳤으면 다시 입력 받음
        if not cmd:
            continue
        
        # q 입력하면 정지 명령 보내고 종료
        if cmd.lower() == "q":
            send_line(ser, "S")
            break
        
        # s 입력하면 정지 명령 보내고 다시 입력 받음
        if cmd.lower() == "s":
            send_line(ser, "S")
            continue

        try:
            dist = float(cmd) # 입력을 실수로 변환
        except ValueError:
            continue

        send_line(ser, f"D {dist:.4f}")

def main():
    try:
        ser = open_serial()
    except Exception:
        return 1

    try:
        interactive_loop(ser)
    except KeyboardInterrupt:
        send_line(ser, "S")
    finally:
        ser.close()

    return 0

if __name__ == "__main__":
    sys.exit(main())