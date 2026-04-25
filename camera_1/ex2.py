import cv2 as cv

img_gray=cv.imread("images/cat on laptop.jpg",cv.IMREAD_GRAYSCALE)

if img_gray is None:
    print("이미지 파일을 읽을 수 없습니다.")
    exit(1)

img_copyed1=img_gray

#print(id(img_gray), id(img_copyed1))

cv.line(img_gray, (0,0), (100,100), 0, 10)
#print(id(img_gray), id(img_copyed1))

cv.imshow("img_gray", img_gray)
cv.imshow("img_copyed1", img_copyed1)

cv.waitKey(0)
cv.destroyAllWindows()