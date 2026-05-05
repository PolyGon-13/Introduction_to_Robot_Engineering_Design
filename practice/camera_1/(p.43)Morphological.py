import cv2
import numpy as np

# 1. 이미지 불러오기
img = cv2.imread("lenna.jpg", cv2.IMREAD_GRAYSCALE)
ret, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

# 2. 커널 생성
kernel = np.ones((5,5), np.uint8)

# 3. 모폴로지 연산 적용
erosion = cv2.erode(binary, kernel, iterations=1)  # 침식
dilation = cv2.dilate(binary, kernel, iterations=1)  # 팽창
opening = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)  # 열기
closing = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)  # 닫기

# 4. 결과 출력
cv2.imshow("Original", binary)
cv2.imshow("Erosion", erosion)
cv2.imshow("Dilation", dilation)
cv2.imshow("Opening", opening)
cv2.imshow("Closing", closing)

cv2.waitKey(0)
cv2.destroyAllWindows()
