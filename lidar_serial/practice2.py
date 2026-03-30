import serial
import time

port = "/dev/ttyUSB0" # USB-시리얼 포트 
baudrate = 460800 # 보드레이트 적용

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
    data = ser.read(5)  # 데이터 패킷 길이 5바이트
    if len(data) == 5:
        quality = data[0] >> 2  # 상위 6비트 품질 값
        angle = ((data[1] >> 1) | (data[2] << 7)) / 64.0  # 0~360도 변환
        distance = ((data[3] | (data[4] << 8))) / 4.0  # mm 거리리 변환

        print(f"Angle: {angle:.2f}°, Distance: {distance:.2f}mm, Quality: {quality}")
