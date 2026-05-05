#!/usr/bin/env python3
# test_motion.py
# 키보드 입력 → 아두이노 V,W 명령 전송 (라파↔아두이노 통신 검증용)

import serial
import sys
import time

ARDU_PORT = "/dev/ttyS0"
ARDU_BAUD = 9600

HELP = """
명령어 예시:
  V0.2,0      → 직진 0.2 m/s
  V-0.2,0     → 후진 0.2 m/s
  V0,1.0      → 제자리 좌회전 1.0 rad/s
  V0,-1.0     → 제자리 우회전 1.0 rad/s
  V0.2,0.5    → 전진하며 좌회전
  S           → 정지
  q           → 종료
""".strip()


def main():
    print(f"Connecting to {ARDU_PORT} @ {ARDU_BAUD}...")
    try:
        ardu = serial.Serial(ARDU_PORT, ARDU_BAUD, timeout=0.1)
    except Exception as e:
        print(f"[ERROR] 시리얼 연결 실패: {e}")
        sys.exit(1)
    time.sleep(2.0)
    print("연결됨.\n")
    print(HELP)
    print()

    try:
        while True:
            cmd = input(">> ").strip()
            if not cmd:
                continue
            if cmd.lower() == 'q':
                break
            # 아두이노로 그대로 전송 (개행 추가)
            msg = (cmd + "\n").encode()
            ardu.write(msg)
            print(f"  sent: {cmd}")
            # 명령은 유지되지만 안전상 5초 후 자동 정지하고 싶으면 아래 주석 해제
            # time.sleep(5.0); ardu.write(b"S\n"); print("  auto-stop")
    except KeyboardInterrupt:
        pass
    finally:
        print("\n정지 후 종료...")
        ardu.write(b"S\n")
        time.sleep(0.2)
        ardu.close()


if __name__ == "__main__":
    main()