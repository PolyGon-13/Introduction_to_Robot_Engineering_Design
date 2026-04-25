import cv2

# 1. 이미지 불러오기 (흑백 변환)
img = cv2.imread("lenna.jpg", cv2.IMREAD_GRAYSCALE)

# 2. 가우시안 블러 적용 (노이즈 제거)
blurred = cv2.GaussianBlur(img, (5,5), 1.4)

# 3. 캐니 에지 검출 적용 (이중 임계값 50, 150)
edges = cv2.Canny(blurred, 50, 150)

# 4. 결과 출력
cv2.imshow("Original", img)
cv2.imshow("Canny Edge Detection", edges)
cv2.waitKey(0)
cv2.destroyAllWindows()
