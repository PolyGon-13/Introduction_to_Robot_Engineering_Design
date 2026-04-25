import cv2

# 이미지 불러오기
img = cv2.imread("lenna.jpg")

# 가우시안 블러 적용 (커널 크기 5x5, 표준편차 1)
blurred = cv2.GaussianBlur(img, (5,5), 1)

# 결과 출력
cv2.imshow("Original", img)
cv2.imshow("Gaussian Blur", blurred)
cv2.waitKey(0)
cv2.destroyAllWindows()
