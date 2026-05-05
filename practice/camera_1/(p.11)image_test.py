import cv2

# 이미지 불러오기
img = cv2.imread('lenna.jpg')  # 'lenna.jpg' 파일을 불러옴

# 이미지 표시
cv2.imshow('Image', img)

# 키 입력 대기 (0: 무한 대기)
cv2.waitKey(0)

# 모든 창 닫기
cv2.destroyAllWindows()
