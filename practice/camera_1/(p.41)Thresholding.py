import cv2

# 1. 이미지 불러오기 & 그레이스케일 변환
img = cv2.imread("lenna.jpg", cv2.IMREAD_GRAYSCALE)

# 2. 기본 임계값 적용 (Threshold = 127)
ret, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

# 3. 결과 출력
cv2.imshow("Original", img)
cv2.imshow("Binary Threshold", binary)
cv2.waitKey(0)
cv2.destroyAllWindows()
