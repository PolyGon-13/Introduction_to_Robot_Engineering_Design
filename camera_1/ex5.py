import cv2 as cv

cap=cv.VideoCapture(0)

if cap.isOpened() == False:
    print("카메라를 열 수 없습니다.")
    exit(1)

while(True):
    ret,img_frame=cap.read()

    if ret==False:
        print("캡쳐 실패")
        break;

    cv.imshow('Color',img_frame)

    key=cv.waitKey(1)

    if key==27:
        break

cap.release()
cv.destroyAllWindows()