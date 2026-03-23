import cv2
import numpy as np

# ArUco 마커 딕셔너리 및 탐지 파라미터 설정
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# **카메라 내부 파라미터 (미리 측정한 값 필요)**
# 실제 사용 시 반드시 카메라 캘리브레이션을 통해 정확한 값을 얻어야 함
camera_matrix = np.array([[700, 0, 320],  # fx, 0, cx
                          [0, 700, 240],  # 0, fy, cy
                          [0, 0, 1]])      # 0, 0, 1
dist_coeffs = np.zeros((4, 1))  # 왜곡 계수

# **마커 실제 크기 (단위: cm)**
marker_size = 10 

# **3D 좌표계 정의 (마커의 실제 크기 기준)**
object_points = np.array([
    [-marker_size / 2, marker_size / 2, 0],   # 좌상단
    [marker_size / 2, marker_size / 2, 0],    # 우상단
    [marker_size / 2, -marker_size / 2, 0],   # 우하단
    [-marker_size / 2, -marker_size / 2, 0]   # 좌하단
], dtype=np.float32)

# 웹캠 열기
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 그레이스케일 변환
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 아루코 마커 탐지
    corners, ids, _ = aruco_detector.detectMarkers(gray)

    if ids is not None:
        for i in range(len(ids)):
            corner = corners[i][0]

            # solvePnP를 사용하여 3D 위치 추정
            success, rvec, tvec = cv2.solvePnP(object_points, corner, camera_matrix, dist_coeffs)

            if success:
                # 마커 중심 좌표 계산
                center_x = int(np.mean(corner[:, 0]))
                center_y = int(np.mean(corner[:, 1]))

                # 마커와 카메라 간 거리 (Z 값)
                distance = round(tvec[2][0], 2)

                # 바운딩 박스 그리기
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

                # x, y 좌표 및 거리 출력
                info_text = f"X: {center_x}, Y: {center_y}, Dist: {distance}cm"
                cv2.putText(frame, info_text, (center_x - 70, center_y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # 화면 출력
    cv2.imshow("Aruco Marker Distance & Position", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
