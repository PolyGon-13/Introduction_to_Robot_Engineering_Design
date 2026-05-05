import serial
import time
port = "/dev/ttyUSB0"  # USB-시리얼 포트 
baudrate = 460800  # 보드레이트 적용

ser = serial.Serial(port, baudrate, timeout=1)

# RESET 요청 패킷 전송 (0xA5 0x40)
scan_request = bytes([0xA5, 0x40])
ser.write(scan_request)
time.sleep(1) # 1초 동안 멈춤

# SCAN 요청 패킷 전송 (0xA5 0x20)
scan_request = bytes([0xA5, 0x20])
ser.write(scan_request)

# 응답 데이터 읽기
while True:
    data = ser.read(10)  # 10바이트씩 읽기
    if data:
        print(data.hex())  # 헥사 값으로 출력
