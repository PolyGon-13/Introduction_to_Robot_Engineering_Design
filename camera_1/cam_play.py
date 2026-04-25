import cv2 as cv

cap=cv.VideoCapture("video/class2_output.avi")

if cap.isOpened()==False:
    print("동영상을 열 수 없습니다.")
    exit(1)

while(True):
    ret,img_frame=cap.read()

    if ret==False:
        print("동영상 파일 읽기 완료")
        break;

    cv.imshow('Color',img_frame)
    
    key=cv.waitKey(25)

    if key==27:
        break

cap.release()
cv.destroyAllWindows()