import sys
import time

import serial


DEFAULT_PORT = "/dev/ttyS0"
DEFAULT_BAUD = 9600
DONE_TIMEOUT_S = 30.0
VERBOSE = False


def open_serial(port: str, baud: int) -> serial.Serial:
    ser = serial.Serial(port, baud, timeout=0.1)
    if VERBOSE:
        print(f"[OK] Serial port opened: {port} @ {baud}")
    time.sleep(2.0)
    ser.reset_input_buffer()
    return ser


def send_line(ser: serial.Serial, line: str) -> None:
    if not line.endswith("\n"):
        line += "\n"
    ser.write(line.encode("ascii"))
    ser.flush()
    if VERBOSE:
        print(f"[SEND] {line.strip()}")


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
                if not text:
                    continue
                if VERBOSE:
                    print(f"[ARDUINO] {text}")
                if text.startswith("DONE"):
                    if VERBOSE:
                        parts = text.split()
                        if len(parts) >= 6:
                            try:
                                enc_r = int(parts[1]); enc_l = int(parts[2])
                                d_r = float(parts[3]); d_l = float(parts[4])
                                d_avg = float(parts[5])
                                print("---------------------------------------------")
                                print(f"  enc_r = {enc_r}")
                                print(f"  enc_l = {enc_l}")
                                print(f"  d_r   = {d_r:.4f} m")
                                print(f"  d_l   = {d_l:.4f} m")
                                print(f"  d_avg = {d_avg:.4f} m")
                                print(f"  diff  = {(d_r - d_l)*1000.0:+.1f} mm  (R - L)")
                                print("---------------------------------------------")
                            except ValueError:
                                pass
                    return True
        else:
            time.sleep(0.01)
    return False


def interactive_loop(ser: serial.Serial) -> None:
    if VERBOSE:
        print()
        print("=== Project 1 : Straight Drive Controller ===")
        print(" enter distance(m)        e.g.  1.5       (forward 1.5 m)")
        print("                                -0.3      (backward 0.3 m)")
        print(" 's'                      emergency stop")
        print(" 'q'                      quit")
        print()

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
                while ser.in_waiting:
                    line = ser.readline().decode("ascii", errors="ignore").strip()
                    if line and VERBOSE:
                        print(f"[ARDUINO] {line}")
                continue

            try:
                dist = float(cmd)
            except ValueError:
                print("[ERROR] invalid number. enter meters (e.g. 1.5) or 's'/'q'")
                continue
            if abs(dist) < 0.001:
                print("[ERROR] distance too small")
                continue

            send_line(ser, f"D {dist:.4f}")
            ok = wait_for_done(ser, DONE_TIMEOUT_S)
            if not ok:
                if VERBOSE:
                    print("[WARN] DONE not received within timeout. sending STOP for safety.")
                send_line(ser, "S")
    except KeyboardInterrupt:
        if VERBOSE:
            print("\n[CTRL-C] sending STOP")
        send_line(ser, "S")


def main() -> int:
    try:
        ser = open_serial(DEFAULT_PORT, DEFAULT_BAUD)
    except Exception as e:
        print(f"[ERROR] couldn't open serial: {e}")
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
