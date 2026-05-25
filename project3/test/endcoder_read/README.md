# Encoder Read Test

Minimal Arduino-to-Raspberry-Pi encoder serial test.

## Files

- `endcoder_read.ino`: Arduino sketch. Reads left/right encoder counts and sends them on `Serial1`.
- `read_encoder_serial.py`: Raspberry Pi script. Reads the serial lines and prints counts/deltas.

## Serial Format

```text
O,<left_count>,<right_count>,<arduino_millis>
```

This matches the odometry format expected by `project3.py`.

## Run On Raspberry Pi

```bash
cd ~/Introduction_to_Robot_Engineering_Design
python3 project3/test/endcoder_read/read_encoder_serial.py --port /dev/ttyS0 --baud 9600
```

If the Arduino is connected by USB instead of GPIO UART, try:

```bash
python3 project3/test/endcoder_read/read_encoder_serial.py --port /dev/ttyACM0 --baud 9600
```

Stop with `Ctrl+C`.
