import cv2
import numpy as np

# 이미지 불러오기
img = cv2.imread("lenna.jpg")

# 3x3 샤프닝 커널 정의
sharpening_kernel = np.array([[ -1, -1,  -1],
                               [-1,  9, -1],
                               [ -1, -1,  -1]])

# 샤프닝 필터 적용
sharpened = cv2.filter2D(img, -1, sharpening_kernel)

# 결과 출력
cv2.imshow("Original", img)
cv2.imshow("Sharpened Image", sharpened)
cv2.waitKey(0)
cv2.destroyAllWindows()
