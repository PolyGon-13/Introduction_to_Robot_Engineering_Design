import cv2

# 1. 이미지 읽기
image = cv2.imread("lenna.jpg")

if image is None:
    print("이미지를 읽을 수 없습니다.")
    exit()

# 2. 이미지 크기 출력
print(f"이미지 크기: {image.shape}") # (height, width, channels)

# 3. 이미지 색상 공간 변환
rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) # BGR -> RGB
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # BGR -> HSV
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) # BGR -> Grayscale

# 4. 이미지 표시
cv2.imshow("Original Image", image)
cv2.imshow("RGBscale Image", rgb_image)
cv2.imshow("HSV Image", hsv_image)
cv2.imshow("Grayscale Image", gray_image)


# 'q'를 누르면 종료
cv2.waitKey(0)
cv2.destroyAllWindows()