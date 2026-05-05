import cv2

# 이미지 불러오기
img = cv2.imread('lenna.jpg')

# 이미지 크기 확인 (높이, 너비, 채널)
h, w, c = img.shape
print(f"원본 이미지 크기: {w}x{h}")

# 크롭할 영역 지정 [y시작:y끝, x시작:x끝]
cropped_img = img[100:300, 100:400]

# 크롭된 이미지 출력
cv2.imshow('Cropped Image', cropped_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
