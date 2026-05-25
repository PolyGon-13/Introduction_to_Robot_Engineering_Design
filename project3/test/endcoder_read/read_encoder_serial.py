#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Print encoder counts sent by endcoder_read.ino.

Expected Arduino line format:
    O,<left_count>,<right_count>,<arduino_millis>
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    serial = None


def parse_args():
    parser = argparse.ArgumentParser(description="Read Arduino encoder counts.")
    parser.add_argument(
        "--port",
        default="/dev/ttyS0",
        help="Serial device connected to Arduino Serial1. Default: /dev/ttyS0",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=9600,
        help="Serial baud rate. Must match BAUD_PI in endcoder_read.ino.",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="Print raw serial lines without parsing.",
    )
    parser.add_argument(
        "--retry-s",
        type=float,
        default=1.0,
        help="Seconds to wait before reopening the port after a serial error.",
    )
    return parser.parse_args()


def parse_encoder_line(line):
    parts = line.strip().split(",")
    if len(parts) != 4 or parts[0] != "O":
        raise ValueError("expected O,left,right,millis")
    left = int(parts[1])
    right = int(parts[2])
    arduino_ms = int(parts[3])
    return left, right, arduino_ms


def open_serial(port, baud):
    ser = serial.Serial(port, baud, timeout=1.0)
    ser.reset_input_buffer()
    return ser


def main():
    if serial is None:
        print("pyserial is not installed. Try: python3 -m pip install pyserial", file=sys.stderr)
        return 2

    args = parse_args()
    ser = None
    prev_left = None
    prev_right = None
    try:
        while True:
            if ser is None:
                try:
                    ser = open_serial(args.port, args.baud)
                except serial.SerialException as exc:
                    print(f"[serial] failed to open {args.port}: {exc}", file=sys.stderr)
                    return 1
                print(f"[open] {args.port} @ {args.baud}")
                print("[wait] rotate wheels by hand, or move the robot slowly")

            try:
                raw = ser.readline()
            except serial.SerialException as exc:
                print(f"[serial] read failed: {exc}", file=sys.stderr)
                print(f"[serial] reopening {args.port} in {args.retry_s:.1f}s", file=sys.stderr)
                try:
                    ser.close()
                except serial.SerialException:
                    pass
                ser = None
                prev_left = None
                prev_right = None
                time.sleep(max(0.1, args.retry_s))
                continue

            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if args.raw:
                print(line)
                continue
            try:
                left, right, arduino_ms = parse_encoder_line(line)
            except ValueError:
                print(f"[skip] {line}")
                continue

            d_left = 0 if prev_left is None else left - prev_left
            d_right = 0 if prev_right is None else right - prev_right
            prev_left = left
            prev_right = right
            host_t = time.strftime("%H:%M:%S")
            print(
                f"{host_t}  arduino_ms={arduino_ms:>10d}  "
                f"L={left:>10d} ({d_left:+5d})  "
                f"R={right:>10d} ({d_right:+5d})"
            )
    except KeyboardInterrupt:
        print("\n[stop]")
    finally:
        if ser is not None:
            ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
