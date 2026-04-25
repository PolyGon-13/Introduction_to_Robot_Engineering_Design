import cv2

# 1. 이미지 불러오기 & 그레이스케일 변환
img = cv2.imread("lenna.jpg")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# 2. 이진화 (Thresholding)
ret, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

# 3. 윤곽선 검출
contours, hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# 4. 윤곽선 그리기
cv2.drawContours(img, contours, -1, (0, 255, 0), 2)  # 초록색 윤곽선

# 5. 결과 출력
cv2.imshow("Contours", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
