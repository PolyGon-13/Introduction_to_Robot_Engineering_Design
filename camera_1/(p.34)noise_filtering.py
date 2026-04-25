import cv2

# 1. 가우시안 노이즈가 추가된 이미지 불러오기
noisy_img = cv2.imread("gaussian_noisy_lenna.jpg")

# 2. 노이즈 제거 필터 적용
avg_blur = cv2.blur(noisy_img, (5,5))                    # 평균 블러
gaussian_blur = cv2.GaussianBlur(noisy_img, (5,5), 0)    # 가우시안 블러

# 3. 결과 출력
cv2.imshow("Noisy Image", noisy_img)
cv2.imshow("Averaging Blur", avg_blur)
cv2.imshow("Gaussian Blur", gaussian_blur)

cv2.waitKey(0)
cv2.destroyAllWindows()
