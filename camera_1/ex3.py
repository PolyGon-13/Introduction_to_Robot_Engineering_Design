import cv2 as cv

img_gray=cv.imread("images/cat on laptop.jpg",cv.IMREAD_GRAYSCALE)
img_copyed1=img_gray.copy()

#print(id(img_gray), id(img_copyed1))

cv.line(img_gray, (0,0), (100,100), 0, 10)

#print(id(img_gray), id(img_copyed1))

cv.imshow("img_gray", img_gray)
cv.imshow("img_copyed1", img_copyed1)

cv.waitKey(0)
cv.destroyAllWindows()