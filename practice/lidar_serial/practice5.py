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
    data = ser.read(5)
    if len(data) != 5:
        continue

    # Start Flag와 Inversed Start Flag 검증
    s_flag = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag):
        continue

    # Check Bit 검증
    check_bit = data[1] & 0x01
    if check_bit != 1:
        continue

    # 품질
    quality = data[0] >> 2

    # 각도 계산
    angle_q6 = ((data[1] >> 1) | (data[2] << 7))
    angle = angle_q6 / 64.0 #각도

    # 거리 계산
    distance_q2 = (data[3] | (data[4] << 8))
    distance = distance_q2 / 4.0  # 거리mm
    if distance < 50:  # 거리가 너무 짧은 경우 노이즈로 간주하고 무시
        continue  
    # 장애물 조건: 각도 ±30도 범위, 거리 300mm 이하
    #*********아래의 if 조건문을 완성하시오***********
    if (distance <= 300) and (angle <= 30 or angle >= 330):
        print(f"Obstacle detected! Angle: {angle:.2f}°, Distance: {distance:.2f} mm")
