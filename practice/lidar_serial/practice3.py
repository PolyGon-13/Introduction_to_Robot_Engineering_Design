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
    if len(data) != 5:
        continue  # 패킷 크기가 맞지 않으면 무시

    # Start Flag와 Inversed Start Flag 검증
    s_flag = data[0] & 0x01  # Start flag
    s_inv_flag = (data[0] & 0x02) >> 1  # Inversed Start flag
    
    if s_inv_flag != (1 - s_flag):  # S̅ 값이 S의 반대인지 확인
        print("Invalid data detected. Skipping packet...")
        continue

    # Check Bit 검증
    check_bit = (data[1] & 0x01)  # C 값
    if check_bit != 1:
        print("Corrupted packet detected. Skipping...")
        continue

    # 품질 값 (상위 6비트 사용)
    quality = data[0] >> 2

    # 각도 계산
    angle_q6 = ((data[1] >> 1) | (data[2] << 7))
    angle = angle_q6 / 64.0  # 0~360도 변환

    # 거리 계산
    distance_q2 = (data[3] | (data[4] << 8))
    distance = distance_q2 / 4.0  # mm 거리 변환

    # 새로운 360° 스캔 시작 감지
    if s_flag == 1:
        print("New 360 scan started")

    print(f"Angle: {angle:.2f}°, Distance: {distance:.2f}mm, Quality: {quality}")
