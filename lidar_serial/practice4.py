import serial
import time
import numpy as np
import cv2
import threading

port = "/dev/ttyUSB0" # USB-시리얼 포트 
baudrate = 460800 # 보드레이트 적용

ser = serial.Serial(port, baudrate, timeout=1)

# OpenCV 맵 설정
MAP_SIZE = 800  # 800x800 픽셀
MAP_CENTER = MAP_SIZE // 2  # 중앙 좌표 (400, 400)
SCALE = 0.05  # 거리 데이터를 픽셀 값으로 변환하는 비율
FADE_RATE = 40  # 점이 서서히 사라지는 속도 (높을수록 빠르게 사라짐)

# 기존 데이터를 유지하면서 서서히 사라지도록 함
map_img = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)

# 스캔 데이터를 저장할 리스트 (스레드 간 공유)
scan_data = []
print_data = []  # 터미널에 출력할 센서 데이터 저장
lock = threading.Lock()  # 멀티스레드에서 데이터 동기화

# LIDAR 초기화 (RESET 후 SCAN 시작)
def initialize_lidar():
    ser.write(bytes([0xA5, 0x40]))  # RESET 요청
    time.sleep(3)
    ser.write(bytes([0xA5, 0x20]))  # SCAN 요청

# OpenCV 가이드 선 및 원 그리기
def draw_guides(img):
    """거리 원(circle guides)과 각도 선(angle lines), 텍스트 추가"""
    for i in range(1, 6):  # 1m ~ 5m 원 추가
        radius = int(i * 1000 * SCALE)  # 1m = 1000mm
        cv2.circle(img, (MAP_CENTER, MAP_CENTER), radius, (50, 50, 50), 1)
        cv2.putText(img, f"{i}m", (MAP_CENTER + radius - 20, MAP_CENTER + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)

    # 주요 각도에 대한 선 추가
    for angle in range(0, 360, 45):
        angle_rad = np.radians(angle)
        x = int(MAP_CENTER + (5000 * SCALE) * np.sin(angle_rad))
        y = int(MAP_CENTER - (5000 * SCALE) * np.cos(angle_rad))  
        cv2.line(img, (MAP_CENTER, MAP_CENTER), (x, y), (50, 50, 50), 1)

        # 각도 텍스트 추가
        text_x = int(MAP_CENTER + (5200 * SCALE) * np.sin(angle_rad))
        text_y = int(MAP_CENTER - (5200 * SCALE) * np.cos(angle_rad))  
        cv2.putText(img, f"{angle} deg", (text_x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)

# **스레드: LIDAR 데이터 읽기**
def read_lidar_data():
    global scan_data, print_data
    while True:
        data = ser.read(5)  # 5바이트씩 읽기
        if len(data) != 5:
            continue  # 패킷 크기가 맞지 않으면 무시

        # Start Flag와 Inversed Start Flag 검증
        s_flag = data[0] & 0x01
        s_inv_flag = (data[0] & 0x02) >> 1
        if s_inv_flag != (1 - s_flag):
            continue  # 데이터 무결성 오류 시 무시

        # Check Bit 검증
        check_bit = data[1] & 0x01
        if check_bit != 1:
            continue  # 잘못된 패킷 무시

        # 품질 값 (상위 6비트 사용)
        quality = data[0] >> 2
        if quality < 15:  # 15미만의 신호 노이즈 제거
            continue  

        # 각도 계산 (0~360도 범위 유지)
        angle_q6 = ((data[1] >> 1) | (data[2] << 7))
        angle = (angle_q6 / 64.0) # 0~360도 변환

        # 거리 계산
        distance_q2 = (data[3] | (data[4] << 8))
        distance = distance_q2 / 4.0  # mm 거리 변환

        if distance < 50:  # 거리가 너무 짧은 경우 노이즈로 간주하고 무시
            continue  

        # X, Y 좌표 변환
        angle_rad = np.radians(angle)
        x = int(MAP_CENTER + (distance * SCALE) * np.sin(angle_rad))
        y = int(MAP_CENTER - (distance * SCALE) * np.cos(angle_rad))

        with lock:  # 동기화
            scan_data.append((angle, distance, x, y))
            print_data.append((angle, distance, x, y))

            # 오래된 데이터 삭제
            if len(scan_data) > 500:
                scan_data.pop(0)
            if len(print_data) > 360:  # 최신 데이터만 유지
                print_data.pop(0)

# **스레드 시작**
initialize_lidar()
lidar_thread = threading.Thread(target=read_lidar_data, daemon=True)
lidar_thread.start()

# **OpenCV GUI 루프**
while True:
    # 기존 점을 유지하면서 서서히 사라지도록 맵에 투명 효과 적용
    map_img = cv2.addWeighted(map_img, 1.0 - (FADE_RATE / 255.0), np.zeros_like(map_img), 0, 0)

    # 거리 원과 각도 선을 미리 그려둠
    draw_guides(map_img)

    # 스캔 데이터를 정렬하여 그리기
    with lock:
        for _, _, x, y in scan_data:
            cv2.circle(map_img, (x, y), 2, (255, 255, 255), -1)

    # LIDAR 중심 위치 강조
    cv2.circle(map_img, (MAP_CENTER, MAP_CENTER), 8, (0, 0, 255), -1)

    # **터미널에 센서 데이터 출력**
    with lock:
        print("\033c", end="")  # 터미널 클리어 (출력 깔끔하게 유지)
        print("RPLIDAR 센서 데이터")
        print("-" * 50)
        for angle, distance, x, y in print_data:
            print(f"Angle: {angle:6.2f}°  |  Distance: {distance:8.2f} mm  |  X: {x:4d}, Y: {y:4d}")

    # OpenCV 창 업데이트
    cv2.imshow("LIDAR Map", map_img)
    if cv2.waitKey(1) & 0xFF == ord('q'):  # 'q' 키를 누르면 종료
        break

# SCAN 종료
ser.write(bytes([0xA5, 0x25]))  # STOP 요청
ser.close()
cv2.destroyAllWindows()
