import cv2 # OpenCV 라이브러리 가져오기

cap = cv2.VideoCapture(0)  
# 0번 카메라 연결(노트북 내장 카메라)
# USB 카메라로 실행하려면 1로 변경

if not cap.isOpened():
    print("카메라를 열 수 없습니다.")
    exit()

while True:
    ret, frame = cap.read()  # 카메라에서 프레임 읽기
    if not ret:
        print("프레임을 읽을 수 없습니다.")
        break

    cv2.imshow("Camera", frame)  # 프레임을 창에 표시

    # 'q'를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()  # 카메라 릴리스
cv2.destroyAllWindows()  # 창 닫기