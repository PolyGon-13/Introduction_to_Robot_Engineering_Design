import cv2
import numpy as np

# ArUco 마커 딕셔너리 및 탐지 파라미터 설정
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# 실제 마커 너비 (cm)
KNOWN_WIDTH = 10
FOCAL_LENGTH = 700  # 초점 거리

# 거리 계산 함수
def estimate_distance(perceived_width):
    D=0.0
    if perceived_width > 0:
        D = (FOCAL_LENGTH * KNOWN_WIDTH) / perceived_width
    #여기에 코드 작성
    return D

# 웹캠 열기
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 그레이스케일 변환
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # ArUco 마커 탐지
    corners, ids, _ = aruco_detector.detectMarkers(gray)

    if ids is not None:
        for i, corner_data in enumerate(corners):
            corner = corner_data[0]
            
            # 중심 좌표 계산 (좌상단과 우하단의 중간값)
            center_x = int((corner[0][0] + corner[2][0]) / 2)  # (x1 + x3) / 2
            center_y = int((corner[0][1] + corner[2][1]) / 2)  # (y1 + y3) / 2

            # 마커 가로 크기 (픽셀 단위)
            perceived_width = corner[2][0] - corner[0][0]

            # 거리 계산
            estimated_distance = estimate_distance(perceived_width)

            # 마커 표시
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)  # 중심점

            # 거리 및 좌표 정보 출력
            info_text = f"X: {center_x}, Y: {center_y}, Dist: {estimated_distance:.2f}cm"
            cv2.putText(frame, info_text, (center_x - 60, center_y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # 화면 출력
    cv2.imshow("Aruco Marker Distance & Position", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
