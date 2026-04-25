import cv2

# 이미지 불러오기
image = cv2.imread("lenna.jpg")

# cv2.putText() 매개변수
text = "Hello, OpenCV!!"
org = (50, 50)
fontFace = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 1
color = (255, 0, 0)
thickness = 2
lineType = cv2.LINE_AA

# 텍스트 추가
cv2.putText(image, text, org, fontFace, fontScale, color, thickness, lineType)

# 결과 출력
cv2.imshow("Text Example", image)
cv2.waitKey(0)
cv2.destroyAllWindows()