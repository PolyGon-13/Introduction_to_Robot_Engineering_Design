import sys
import time

import serial


DEFAULT_PORT = "/dev/ttyS0"
DEFAULT_BAUD = 9600
DONE_TIMEOUT_S = 30.0


def open_serial(port: str, baud: int) -> serial.Serial:
    ser = serial.Serial(port, baud, timeout=0.1)
    time.sleep(2.0)
    ser.reset_input_buffer()
    return ser


def send_line(ser: serial.Serial, line: str) -> None:
    if not line.endswith("\n"):
        line += "\n"
    ser.write(line.encode("ascii"))
    ser.flush()


def wait_for_done(ser: serial.Serial, timeout_s: float = DONE_TIMEOUT_S) -> bool:
    t0 = time.time()
    buf = b""
    while time.time() - t0 < timeout_s:
        chunk = ser.read(128)
        if chunk:
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("ascii", errors="ignore").strip()
                if text.startswith("DONE"):
                    return True
        else:
            time.sleep(0.01)
    return False


def interactive_loop(ser: serial.Serial) -> None:
    try:
        while True:
            try:
                cmd = input(">> ").strip()
            except EOFError:
                break
            if not cmd:
                continue
            if cmd.lower() == "q":
                send_line(ser, "S")
                break
            if cmd.lower() == "s":
                send_line(ser, "S")
                time.sleep(0.2)
                ser.reset_input_buffer()
                continue

            try:
                dist = float(cmd)
            except ValueError:
                continue
            if abs(dist) < 0.001:
                continue

            send_line(ser, f"D {dist:.4f}")
            if not wait_for_done(ser, DONE_TIMEOUT_S):
                send_line(ser, "S")
    except KeyboardInterrupt:
        send_line(ser, "S")


def main() -> int:
    try:
        ser = open_serial(DEFAULT_PORT, DEFAULT_BAUD)
    except Exception:
        return 1

    try:
        interactive_loop(ser)
    finally:
        try:
            ser.close()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
