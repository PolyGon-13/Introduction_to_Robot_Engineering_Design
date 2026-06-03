import os
import numpy as np
import cv2

# ===== 설정 =====
CHECKERBOARD = (8, 8)          # 체커보드 내부 코너 개수
SQUARE_SIZE = 21.0             # 사각형 한 변 크기 (mm)
MIN_IMAGES = 15                # 캘리브레이션에 필요한 최소 캡처 수
CAMERA_INDEX = 0               # 시스템 기본 카메라

# 결과 저장 디렉토리 (이 스크립트가 위치한 디렉토리)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# 코너 검출 종료 기준 (cornerSubPix / calibrate)
SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# ===== 3D 오브젝트 포인트 준비 (Z=0 평면) =====
# (0,0,0), (1,0,0), ... 형태로 만든 뒤 실제 크기(mm)를 곱함
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

# 캡처 성공한 프레임들의 포인트 누적 저장 리스트
objpoints = []  # 3D 실세계 좌표
imgpoints = []  # 2D 이미지 좌표

capture_count = 0
last_status = ""        # 마지막 검출 상태 텍스트
status_timer = 0        # 상태 텍스트 표시 잔여 프레임


def run_calibration(image_size):
    """누적된 포인트로 캘리브레이션 실행 후 결과 저장/출력."""
    flags = cv2.CALIB_RATIONAL_MODEL  # k1~k6, p1, p2 (8개 왜곡 계수)

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None, flags=flags
    )

    # ===== 재투영 오차 계산 =====
    total_error = 0.0
    for i in range(len(objpoints)):
        projected, _ = cv2.projectPoints(
            objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs
        )
        error = cv2.norm(imgpoints[i], projected, cv2.NORM_L2) / len(projected)
        total_error += error
    reprojection_error = total_error / len(objpoints)

    # ===== 결과 저장 =====
    mtx_path = os.path.join(SCRIPT_DIR, "camera_matrix.npy")
    dist_path = os.path.join(SCRIPT_DIR, "dist_coeffs.npy")
    txt_path = os.path.join(SCRIPT_DIR, "calibration_result.txt")

    np.save(mtx_path, camera_matrix)
    np.save(dist_path, dist_coeffs)

    with open(txt_path, "w", encoding="utf-8") as f:
        f.write("===== Camera Calibration Result =====\n")
        f.write(f"Image size (w, h): {image_size}\n")
        f.write(f"Checkerboard inner corners: {CHECKERBOARD}\n")
        f.write(f"Square size: {SQUARE_SIZE} mm\n")
        f.write(f"Number of images used: {len(objpoints)}\n\n")
        f.write("Camera matrix:\n")
        f.write(np.array2string(camera_matrix) + "\n\n")
        f.write("Distortion coefficients (k1, k2, p1, p2, k3, k4, k5, k6):\n")
        f.write(np.array2string(dist_coeffs) + "\n\n")
        f.write(f"Reprojection error: {reprojection_error}\n")

    # ===== 터미널 출력 =====
    print("\n===== Camera Calibration Result =====")
    print(f"Number of images used: {len(objpoints)}")
    print("\nCamera matrix:")
    print(camera_matrix)
    print("\nDistortion coefficients (k1, k2, p1, p2, k3, k4, k5, k6):")
    print(dist_coeffs)
    print(f"\nReprojection error: {reprojection_error}")
    print(f"\nSaved:\n  {mtx_path}\n  {dist_path}\n  {txt_path}")

    return reprojection_error


def main():
    global capture_count, last_status, status_timer

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"Error: Cannot open camera (index {CAMERA_INDEX})")
        return

    image_size = None  # (width, height)

    print("=== Camera Calibration ===")
    print("[SPACE] : 체커보드 검출 시도")
    print(f"[c]     : 캘리브레이션 실행 (최소 {MIN_IMAGES}장 필요)")
    print("[q]     : 종료")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to grab frame")
            break

        image_size = (frame.shape[1], frame.shape[0])
        display = frame.copy()

        # 좌상단에 현재 캡처 성공 횟수 표시
        cv2.putText(display, f"Captured: {capture_count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # 상태 텍스트 표시 (일정 프레임 동안)
        if status_timer > 0:
            color = (0, 0, 255) if "Failed" in last_status or "Need" in last_status \
                else (0, 255, 0)
            cv2.putText(display, last_status, (10, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            status_timer -= 1

        cv2.imshow("Calibration", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        elif key == ord(' '):
            # 체커보드 코너 검출 시도
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(
                gray, CHECKERBOARD,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
            )

            if found:
                # 서브픽셀 정밀도로 코너 위치 보정
                corners_refined = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1), SUBPIX_CRITERIA
                )

                objpoints.append(objp.copy())
                imgpoints.append(corners_refined)
                capture_count += 1

                # 검출된 코너를 그려서 시각적으로 확인
                vis = frame.copy()
                cv2.drawChessboardCorners(vis, CHECKERBOARD, corners_refined, found)
                cv2.imshow("Calibration", vis)
                cv2.waitKey(500)

                last_status = f"Detected! ({capture_count})"
                status_timer = 30
                print(f"Detected corners. Total captured: {capture_count}")
            else:
                last_status = "Detection Failed"
                status_timer = 30
                print("Detection Failed")

        elif key == ord('c'):
            # 캘리브레이션 실행
            if capture_count < MIN_IMAGES:
                last_status = f"Need more images (current: {capture_count})"
                status_timer = 60
                print(last_status)
            else:
                print("\nRunning calibration...")
                run_calibration(image_size)
                last_status = "Calibration done!"
                status_timer = 90

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
