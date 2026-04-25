import cv2 as cv
import numpy as np

img_color = cv.imread("images/apple.png", cv.IMREAD_COLOR)

height,width=img_color.shape[:2]

#print(height,width)

img_gray=np.zeros((height,width),np.uint8)
#print(img_gray)

for y in range(0,height):
    for x in range(0,width):
        b=img_color.item(y,x,0)
        g=img_color.item(y,x,1)
        r=img_color.item(y,x,2)

        gray=int(r*0.2126+g*0.7152+b*0.0722) # 가중치 적용 (Luminance 방식)

        img_gray[y,x]=gray

img_result=cv.cvtColor(img_gray,cv.COLOR_GRAY2BGR) # 컬러를 표현하기 위해 grayscaled에서 bgr형식으로 변환

for y in range(150,201):
    for x in range(200,251):
        img_result[y,x,0]=0 # Blue 성분 제거
        img_result[y,x,1]=255 # Green 성분 최대
        img_result[y,x,2]=0 # 

cv.imshow('color',img_color)
cv.imshow('result',img_result)

cv.waitKey(0)
cv.destroyAllWindows()