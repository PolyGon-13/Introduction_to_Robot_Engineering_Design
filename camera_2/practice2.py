import cv2

# Open the camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("Camera Not Found.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to return image")
        break

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply binary threshold
    _, binary = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY)

    # Apply morphological operation to clean up the binary image
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    # Find contours
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
        area=cv2.contourArea(c)
        if area < 4000:
            continue

        # Draw the contour
        cv2.drawContours(frame, [c], -1, (255, 0, 0), 2)
        
        x, y, w, h = cv2.boundingRect(c)
        aspect_ratio = float(w) / h
        if area > 4000 and 0.75 < aspect_ratio < 1.3 and len(c) >= 5:
            ellipse = cv2.fitEllipse(c)
            cv2.ellipse(frame, ellipse, (0, 255, 0), 2)

    # Show the result
    cv2.imshow("Contour & Ellipse Detection", frame)

    # Exit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release camera and close windows
cap.release()
cv2.destroyAllWindows()
