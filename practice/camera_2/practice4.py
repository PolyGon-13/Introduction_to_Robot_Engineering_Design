import cv2
import numpy as np
import imutils 
import time

# 모델 파일 경로 설정
protoPath = "deploy.prototxt"
modelPath = "res10_300x300_ssd_iter_140000.caffemodel"

detector = cv2.dnn.readNetFromCaffe(protoPath, modelPath)

# Haarcascade 파일 로드
eye_cascade = cv2.CascadeClassifier("haarcascade_eye.xml")
nose_cascade = cv2.CascadeClassifier("haarcascade_mcs_nose.xml")
mouth_cascade = cv2.CascadeClassifier("haarcascade_mcs_mouth.xml")

# 웹캠 열기
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    print("웹캠을 열 수 없습니다.")
    exit()

time.sleep(2.0)  # 카메라 워밍업 시간

while True:
    ret, frame = cap.read()
    if not ret:
        print("카메라에서 프레임을 가져올 수 없습니다.")
        break

    frame = imutils.resize(frame, width=600)
    (h, w) = frame.shape[:2]

    # 이미지를 Blob 형식으로 변환
    imageBlob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0), swapRB=False, crop=False)

    # 얼굴 감지
    detector.setInput(imageBlob)
    detections = detector.forward()

    for i in range(0, detections.shape[2]):
        confidence = detections[0, 0, i, 2]

        if confidence > 0.5:  # 신뢰도가 50% 이상인 경우만 처리
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")

            # 좌표 표시 (박스 왼쪽 위 모서리와 오른쪽 아래 모서리) 프레임 좌측 위가 0,0임
            cv2.putText(frame, f"({startX}, {startY})", (startX, startY + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(frame, f"({endX}, {endY})", (endX - 100, endY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # 얼굴 영역 표시
            face = frame[startY:endY, startX:endX]
            (fH, fW) = face.shape[:2]

            if fW < 20 or fH < 20:  # 얼굴 크기가 너무 작으면 무시
                continue

            #아래는 사각형 박스 처리
            y = startY - 10 if startY - 10 > 10 else startY + 10
            cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 0, 255), 2)

            gray_face = cv2.cvtColor(face, cv2.COLOR_BGR2GRAY)

            # 눈 탐지
            eyes = eye_cascade.detectMultiScale(gray_face, scaleFactor=1.1, minNeighbors=10, minSize=(20, 20))
            for (ex, ey, ew, eh) in eyes:
                cv2.rectangle(face, (ex, ey), (ex + ew, ey + eh), (255, 0, 0), 2)  # 파란색
                cv2.putText(face, "Eye", (ex, ey - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

            # 코 탐지
            nose = nose_cascade.detectMultiScale(gray_face, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
            for (nx, ny, nw, nh) in nose:
                cv2.rectangle(face, (nx, ny), (nx + nw, ny + nh), (0, 255, 255), 2)  # 노란색
                cv2.putText(face, "Nose", (nx, ny - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # 입 탐지
            mouth = mouth_cascade.detectMultiScale(gray_face, scaleFactor=1.1, minNeighbors=15, minSize=(30, 30))
            for (mx, my, mw, mh) in mouth:
                my = int(my + 0.4 * mh)  # 입의 위치 보정
                cv2.rectangle(face, (mx, my), (mx + mw, my + mh), (0, 0, 255), 2)  # 빨간색
                cv2.putText(face, "Mouth", (mx, my - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    # 결과 화면 출력
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

# 자원 해제
cap.release()
cv2.destroyAllWindows()
