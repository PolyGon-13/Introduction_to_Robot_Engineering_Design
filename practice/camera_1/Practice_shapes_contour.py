import cv2

# 1. 이미지 불러오기
img = cv2.imread("shapes.jpg")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# 2. 이미지 전처리
blurred = cv2.GaussianBlur(gray, (5, 5), 0)
_, binary = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY_INV)

# 3. 윤곽선 검출
contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# 4. 도형 개수 세기
triangle_count = 0
rectangle_count = 0
circle_count = 0

for contour in contours:
    epsilon = 0.01 * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)
    num_vertices = len(approx)

    if num_vertices == 3:
        triangle_count += 1
    elif num_vertices == 4:
        rectangle_count += 1
    else:
        circle_count += 1

# 5. 결과 화면에 출력
cv2.drawContours(img, contours, -1, (0, 255, 0), 2)  # 초록색 윤곽선

cv2.putText(img, f"Triangles: {triangle_count}",    (10, 30),  cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
cv2.putText(img, f"Rectangles: {rectangle_count}",  (10, 60),  cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
cv2.putText(img, f"Circles: {circle_count}",        (10, 90),  cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

# 6. 결과 보여주기
cv2.imshow("Shape Contour Detection", img)
cv2.waitKey(0)
cv2.destroyAllWindows()