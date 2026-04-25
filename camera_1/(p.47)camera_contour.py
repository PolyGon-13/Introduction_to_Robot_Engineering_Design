import cv2

# 카메라 열기
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("카메라를 열 수 없습니다.")
    exit()
    
while True:
    ret, frame = cap.read()
    if not ret:
        print("영상을 가져올 수 없습니다.")
        break
    
    # 그레이 스케일 변환
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # 이진화
    _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    
    # Contour Detection
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # 윤곽선 그리기
    cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)
    
    # 화면에 영상 출력
    cv2.imshow('Contours', frame)
    
    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 자원 해제
cap.release()
cv2.destroyAllWindows()