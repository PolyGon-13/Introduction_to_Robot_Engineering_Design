import cv2

# 이미지 불러오기
img = cv2.imread("lenna.jpg")

# 평균 블러링 적용 (3x3 커널)
blurred = cv2.blur(img, (3, 3))

# 결과 출력
cv2.imshow("Original", img)
cv2.imshow("Averaging Blur", blurred)
cv2.waitKey(0)
cv2.destroyAllWindows()
