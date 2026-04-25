import cv2
import numpy as np

# 1. 빈 이미지 생성 (흰색 배경)
img = 255 * np.ones((500, 800, 3), dtype=np.uint8)

# 2. 선(Line) 그리기
cv2.line(img, (50, 50), (750, 50), (255, 0, 0), 5)  # 파란색, 두께 5

# 3. 사각형(Rectangle) 그리기
cv2.rectangle(img, (100, 100), (300, 300), (0, 255, 0), 3)  # 초록색, 두께 3

# 4. 원(Circle) 그리기
cv2.circle(img, (500, 200), 50, (0, 0, 255), -1)  # 빨간색, 채우기 (-1)

# 5. 타원(Ellipse) 그리기
cv2.ellipse(img, (400, 350), (100, 50), 0, 0, 360, (128, 0, 128), 3)  # 보라색 타원

# 6. 다각형(Polygon) 그리기
pts = np.array([[600, 400], [700, 300], [800, 400]], np.int32)
pts = pts.reshape((-1, 1, 2))  # 다각형 형태로 변환
cv2.polylines(img, [pts], isClosed=True, color=(0, 255, 255), thickness=3)  # 노란색

# 7. 결과 출력
cv2.imshow("Shapes on Image", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
