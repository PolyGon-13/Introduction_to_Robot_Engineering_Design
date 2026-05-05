import serial
import time

try:
    ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
    print("[OK] Serial port opened.")
except Exception as e:
    print(f"[ERROR] Couldn't open serial: {e}")
    exit()

time.sleep(2)

while True:
    input_command = input("enter v w : ")
    try:
        v, w = map(float, input_command.split())
        data = f"{v} {w}\n"
        ser.write(data.encode())
        print(f"[SEND] {data.strip()} to Arduino")
    except ValueError:
        print("[ERROR] Invalid input")
