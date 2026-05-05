import cv2
import numpy as np

# 웹캠 열기
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# HSV 색상 범위 설정 (형광 녹색 & 형광 노랑 계열)
lower_green = np.array([30, 100, 100])
upper_green = np.array([80, 255, 255])

lower_yellow = np.array([20, 100, 100])
upper_yellow = np.array([40, 255, 255])

# 거리 측정 파라미터 (비례 관계 이용)
KNOWN_WIDTH =10.0  # W 형광 사각형 크기(cm)
FOCAL_LENGTH = 700  # F 카메라 초점 거리 (보정 가능)
#perceived_width : P

# 거리 추정 함수 (비례 관계 활용)
def estimate_distance(perceived_width):
    D=0.0
    if perceived_width > 0:
        D = (FOCAL_LENGTH * KNOWN_WIDTH) / perceived_width
    #여기에 코드 작성
    return D

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 형광 조끼 검출 (초록 & 노랑)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    mask = cv2.bitwise_or(mask_green, mask_yellow)

    # 컨투어 검출
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        if cv2.contourArea(cnt) > 1000:
            x, y, w, h = cv2.boundingRect(cnt)

            # 중심 좌표 계산
            center_x = x + w // 2
            center_y = y + h // 2

            # 거리 추정 (바운딩 박스 너비 기반)
            estimated_distance = estimate_distance(w)

            # 바운딩 박스 및 중심점 표시
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

            # 좌표 및 거리 표시
            info_text = f"X: {center_x}, Y: {center_y}, Dist: {estimated_distance:.2f}cm"
            cv2.putText(frame, info_text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # 화면 출력
    cv2.imshow("Vest Detection & Distance", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
