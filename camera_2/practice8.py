import cv2

# 카메라 열기
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 최신 OpenCV에서는 ArUco 딕셔너리를 아래와 같이 사용해야 함
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # ArUco 마커 검출
    corners, ids, _ = aruco_detector.detectMarkers(frame)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)  # 마커 주변 사각형 및 ID 표시

        for i, corner in enumerate(corners):
            x, y = int(corner[0][0][0]), int(corner[0][0][1])
            cv2.putText(frame, f"ID: {ids[i][0]}", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow("ArUco Marker Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
