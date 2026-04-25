import cv2 as cv
import numpy as np

# 이미지를 BGR 모드로 읽어오기
img_apple = cv.imread("images/apple.png", cv.IMREAD_COLOR)

# 이미지가 제대로 로드되지 않았다면 프로그램을 종료
if img_apple is None:
    exit(1)

result_image = img_apple.copy() # 이미지 복사본 제작
hsv_image = cv.cvtColor(img_apple, cv.COLOR_BGR2HSV) # HSV 색 공간으로 변환

# HSV (색상, 채도, 명도)
# 빨간색 범위 1 (0~10) : 어두운 빨간색부터 밝은 빨간색까지 범위 지정
lower_red1 = np.array([0, 100, 5])
upper_red1 = np.array([10, 255, 255])
mask_red1 = cv.inRange(hsv_image, lower_red1, upper_red1)

# 빨간색 범위 2 (160~180) : 진한 빨간색 영역 범위 지정
lower_red2 = np.array([160, 100, 5])
upper_red2 = np.array([180, 255, 255])
mask_red2 = cv.inRange(hsv_image, lower_red2, upper_red2)

# 위의 두 범위 중 하나라도 해당하면 빨간색으로 간주
full_red_mask = cv.bitwise_or(mask_red1, mask_red2)

# 연산에 사용할 도구 제작 (모든 값이 1로 채워진 15X15 크기의 사각형 행렬)
kernel = np.ones((15, 15), np.uint8)

# cv.morphologyEx(입력 이미지, 연산종류, 도구)
# 모폴로지 닫기 연산으로 흰색 반사광 구멍 매우기
full_red_mask = cv.morphologyEx(full_red_mask, cv.MORPH_CLOSE, kernel)

h, w = full_red_mask.shape # 이미지의 높이와 너비 구하기
roi_mask = np.zeros_like(full_red_mask) # 원본과 크기가 동일한 검은색 도화지 제작

# 상단 30%를 제외한 나머지 영역만 흰색(255)으로 채우기
y_start = int(h * 0.428) # 기준점 설정
roi_mask[y_start:h, :] = 255

# 빨간색이면서 상단 30%를 제외한 나머지 영역만 남김
final_mask = cv.bitwise_and(full_red_mask, roi_mask)

# 최종 마스크에서 흰색인 위치의 픽셀들만 골라 BGR 값을 초록색으로 변환
result_image[final_mask == 255] = [0, 255, 0]

cv.imshow('apple', result_image) # 결과 영상을 'apple'이라는 이름의 창에 띄움

# 아무 키나 누를 때까지 창을 유지하다가 키 입력 시 모든 창을 닫음
cv.waitKey(0)
cv.destroyAllWindows()