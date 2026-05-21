#!/usr/bin/env python3
# keyboard_drive.py
# WASD 키보드 수동 조종 코드
# W: 전진
# S: 후진
# A: 좌회전
# D: 우회전
# Space 또는 X: 정지
# Q: 종료

import serial
import time
import sys
import termios
import tty
import select


# ─────────────── 사용자 설정 ───────────────
ARDU_PORT = "/dev/ttyS0"
ARDU_BAUD = 9600

# 전진/후진 속도
LINEAR_V = 0.18

# 회전 각속도
ANGULAR_W = 0.55

# 키 입력이 끊겼을 때 자동 정지까지 걸리는 시간
# 키를 꾹 누르고 있으면 계속 움직이고, 손을 떼면 자동 정지
KEY_TIMEOUT_S = 0.35

# 명령 전송 주기
SEND_PERIOD_S = 0.05


def get_key(timeout=0.05):
    """
    timeout 시간 안에 키 입력이 있으면 1글자 반환.
    없으면 None 반환.
    """
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)

    if rlist:
        return sys.stdin.read(1)

    return None


def send_vw(ardu, v, w):
    cmd = f"V{v:.3f},{w:.3f}\n"
    ardu.write(cmd.encode())


def send_stop(ardu):
    ardu.write(b"S\n")


def print_help():
    print()
    print("========== WASD 수동 조종 ==========")
    print("W : 전진")
    print("S : 후진")
    print("A : 좌회전")
    print("D : 우회전")
    print("Space 또는 X : 정지")
    print("Q : 종료")
    print("===================================")
    print()
    print("[INFO] 키를 꾹 누르면 움직이고, 손을 떼면 자동 정지합니다.")
    print()


def main():
    ardu = serial.Serial(ARDU_PORT, ARDU_BAUD, timeout=0.1)
    time.sleep(2.0)

    old_settings = termios.tcgetattr(sys.stdin)

    v = 0.0
    w = 0.0
    last_key_time = 0.0
    last_send_time = 0.0

    try:
        tty.setcbreak(sys.stdin.fileno())

        send_stop(ardu)
        print_help()
        print("[READY] 키보드 조종 시작")

        while True:
            now = time.time()
            key = get_key(timeout=0.01)

            if key is not None:
                key = key.lower()
                last_key_time = now

                if key == "w":
                    v = LINEAR_V
                    w = 0.0
                    print("[CMD] 전진")

                elif key == "s":
                    v = -LINEAR_V
                    w = 0.0
                    print("[CMD] 후진")

                elif key == "a":
                    v = 0.0
                    w = ANGULAR_W
                    print("[CMD] 좌회전")

                elif key == "d":
                    v = 0.0
                    w = -ANGULAR_W
                    print("[CMD] 우회전")

                elif key == "x" or key == " ":
                    v = 0.0
                    w = 0.0
                    send_stop(ardu)
                    print("[CMD] 정지")

                elif key == "q":
                    print("[INFO] 종료")
                    break

            # 키 입력이 일정 시간 없으면 자동 정지
            if now - last_key_time > KEY_TIMEOUT_S:
                v = 0.0
                w = 0.0

            # 일정 주기로 명령 전송
            if now - last_send_time >= SEND_PERIOD_S:
                if v == 0.0 and w == 0.0:
                    send_stop(ardu)
                else:
                    send_vw(ardu, v, w)

                last_send_time = now

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C 종료")

    finally:
        send_stop(ardu)
        time.sleep(0.2)
        ardu.close()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("[INFO] 모터 정지 및 시리얼 종료")


if __name__ == "__main__":
    main()
