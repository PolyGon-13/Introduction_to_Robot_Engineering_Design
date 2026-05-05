#!/usr/bin/env python3
# 이 파일을 리눅스/라즈베리파이에서 직접 실행할 수 있게 하는 shebang입니다.
# maze_runner.py
# 파일 이름을 표시하는 헤더 주석입니다.
# RPLiDAR C1 + corridor 기하 기반 회피 주행
# RPLiDAR 정면 스캔을 이용해 통과 가능한 방향을 고르고 Arduino에 V,W 명령을 보내는 코드입니다.

import serial  # Raspberry Pi와 라이다/아두이노를 시리얼 통신으로 연결하기 위한 모듈입니다.
import time  # 워밍업, 주기 제어, 타임아웃 계산에 사용하는 시간 모듈입니다.
import math  # sin/cos/atan/부호 등 기본 수학 함수를 쓰기 위한 모듈입니다.
import threading  # 라이다 수신을 별도 스레드에서 계속 돌리기 위한 모듈입니다.
import numpy as np  # 라이다 점군 배열 처리와 percentile/clip 계산에 사용하는 모듈입니다.

# ─────────────── 사용자 튜닝 파라미터 ───────────────
LIDAR_PORT = "/dev/ttyUSB0"  # RPLiDAR가 연결된 라즈베리파이 포트입니다.
LIDAR_BAUD = 460800  # RPLiDAR C1 통신 속도입니다.
ARDU_PORT = "/dev/ttyS0"  # Arduino Serial1과 연결된 라즈베리파이 UART 포트입니다.
ARDU_BAUD = 9600  # Arduino 쪽 명령 수신 통신 속도입니다.

# 라이다 캘리브레이션 (확정값)
ANGLE_OFFSET_DEG = +1.54  # 라이다 장착 오차를 보정하기 위해 모든 각도에 더하는 값입니다.
DIST_OFFSET_MM = 0.0  # 라이다 거리 측정값에 더할 거리 보정값입니다.

# 좌표계: +각도 = 우측, -각도 = 좌측 (CW)
# check.py 결과 기준으로 왼쪽 앞 장애물은 음수 각도/y, 오른쪽 앞 장애물은 양수 각도/y로 해석합니다.

# 라이다 전처리 파라미터
FOV_HALF_DEG = 90  # 정면 기준 좌우 90도, 총 180도만 주행 판단에 사용합니다.
DISP_CLAMP = 3.0  # [m] 유효 거리가 없거나 너무 먼 경우 3m로 제한해서 계산합니다.

# 속도/조향 정책: 정상 회피 중에는 정지/감속/후진 없이 조향으로만 피함
V_CRUISE = 0.14  # [m/s] 기본 전진 속도입니다.
W_MAX = 1.10  # [rad/s] 아두이노에 보낼 최대 각속도 제한입니다.
ROBOT_WIDTH = 0.20  # [m] 바퀴까지 포함한 로봇 전체 폭입니다.
ROBOT_LENGTH = 0.17  # [m] 로봇의 앞뒤 길이입니다.
ROBOT_HALF_WIDTH = ROBOT_WIDTH * 0.5  # [m] 충돌 판정에 쓰는 로봇 반폭입니다.
ROBOT_FRONT_FROM_LIDAR = ROBOT_LENGTH * 0.5  # [m] 라이다에서 로봇 전방 끝까지의 대략적인 거리입니다.
SAFETY_MARGIN = 0.080  # [m] 로봇 반폭에 추가하는 안전 여유입니다.
CORRIDOR_HALF = ROBOT_HALF_WIDTH + SAFETY_MARGIN  # [m] 로봇이 지나갈 가상 통로의 반폭입니다.
DETECT_MARGIN = 0.140  # [m] 장애물 후보를 더 넓게 잡기 위한 감지 여유입니다.
DETECT_CORRIDOR_HALF = ROBOT_HALF_WIDTH + DETECT_MARGIN  # [m] 정면 장애물 후보 영역의 반폭입니다.
CORRIDOR_LOOKAHEAD = 1.25  # [m] 앞으로 몇 m까지 통로 충돌을 볼지 정하는 거리입니다.
MIN_OBS_X = ROBOT_FRONT_FROM_LIDAR  # [m] 라이다 바로 아래/로봇 몸체 내부 점을 제외하기 위한 최소 x입니다.
BLOCK_MIN_POINTS = 3  # 장애물/통로 막힘으로 인정할 최소 라이다 점 개수입니다.
OBSTACLE_X_WINDOW = 0.24  # [m] 가장 가까운 장애물 주변 cluster를 묶는 x 방향 범위입니다.
FRONT_HALF_DEG = 24.0  # 정면 거리 front를 계산할 때 사용하는 좌우 각도 범위입니다.
HEADING_MAX_DEG = 62.0  # 회피 목표 heading의 최대 각도입니다.
HEADING_SAMPLE_DEG = 4.0  # 후보 heading을 몇 도 간격으로 샘플링할지 정합니다.
HEADING_MIN_CLEAR = 0.40  # [m] 후보 경로가 이보다 가까이 막히면 큰 패널티를 줍니다.
TARGET_CLEAR_DIST = 0.95  # [m] 직진 또는 후보 경로가 충분히 안전하다고 보는 목표 여유 거리입니다.
K_HEADING = 1.45  # 목표 heading 각도를 실제 각속도 명령으로 바꾸는 비례 gain입니다.
HEADING_DEADBAND_DEG = 4.0  # 이 각도 이내의 작은 heading은 사실상 직진으로 취급합니다.
FORWARD_COST = 0.60  # 평상시 큰 회전각을 싫어하도록 주는 비용 가중치입니다.
PREV_HEADING_COST = 0.75  # 직전 heading과 급격히 달라지는 것을 막기 위한 비용 가중치입니다.
CLEARANCE_COST = 1.80  # 경로 여유거리가 부족할 때 비용을 키우는 가중치입니다.
SWITCH_SIGN_COST = 1.40  # 좌/우 회피 방향을 바꿀 때 추가하는 평상시 비용입니다.
CLEAR_DECISIVE_MARGIN = 0.20  # [m] 좌/우 여유거리 차이가 이보다 크면 더 열린 쪽을 우선합니다.
FRONT_URGENT_DIST = 0.75  # [m] 이 거리 안쪽부터 장애물이 가깝다고 보고 urgency를 올립니다.
FRONT_CRITICAL_DIST = 0.32  # [m] 이 거리 안쪽은 urgency를 최대로 봅니다.
FORWARD_COST_URGENT = 0.25  # 급한 상황에서 회전각 비용을 낮춰 더 과감히 꺾게 합니다.
PREV_HEADING_COST_URGENT = 0.15  # 급한 상황에서 직전 heading 유지 비용을 낮춥니다.
CLEARANCE_COST_URGENT = 5.00  # 급한 상황에서 경로 여유거리 부족을 강하게 벌점 처리합니다.
SWITCH_SIGN_COST_URGENT = 3.00  # 급한 상황에서 좌/우 방향 전환을 더 무겁게 보는 비용입니다.
MIN_AVOID_W_URGENT = 0.50  # [rad/s] 급한 상황에서 최소한 이 정도는 회전하도록 하는 각속도 기준입니다.
TURN_BALANCE_CLAMP = 0.75  # [rad], 오른쪽 누적 회전이 +가 되도록 누적값을 제한합니다.
TURN_BALANCE_DECAY_PER_S = 0.22  # 회피가 아닐 때 누적 회전량을 초당 얼마나 줄일지 정합니다.
SIDE_GUARD_DIST = 0.45  # [m] 측면 장애물/벽이 이보다 가까우면 반대쪽으로 밀어냅니다.
SIDE_GUARD_GAIN = 0.9  # 측면 근접 정도를 보정 각속도로 바꾸는 gain입니다.
SIDE_GUARD_W_MAX = 0.30  # [rad/s] 측면 보호 보정 각속도의 최대값입니다.
W_DEADBAND = 0.05  # [rad/s] 너무 작은 각속도 명령은 0으로 만들어 모터 떨림을 줄입니다.
W_SMOOTH_ALPHA = 0.25  # 평상시 각속도 명령 저역통과 필터 계수입니다.
W_RATE_LIMIT_STEP = 0.10  # [rad/s] per loop 평상시 한 루프에서 각속도가 바뀔 수 있는 최대량입니다.
W_URGENT_SMOOTH_ALPHA = 0.50  # 급한 상황에서 더 빠르게 각속도 목표를 따라가게 하는 필터 계수입니다.
W_URGENT_RATE_LIMIT_STEP = 0.22  # [rad/s] per loop 급한 상황에서 허용하는 각속도 변화량입니다.
SCAN_HOLD_S = 0.30  # [s] 짧은 스캔 누락은 직전 명령을 유지하는 시간입니다.
LOOP_DT_S = 0.05  # [s] 메인 제어 루프의 목표 주기입니다.

V_SCALE_FRONT_NEAR = 0.25  # [m] 전방 거리가 이 값에 가까우면 cruise_velocity가 작아집니다.
V_SCALE_FRONT_FAR = 0.70  # [m] 전방 거리가 이 값 이상이면 속도 제한 영향이 작아집니다.
V_SCALE_CLEAR_NEAR = 0.30  # [m] 경로 여유가 이 값에 가까우면 cruise_velocity가 작아집니다.
V_SCALE_CLEAR_FAR = 0.80  # [m] 경로 여유가 이 값 이상이면 속도 제한 영향이 작아집니다.

NO_PATH_CLEAR_THRESH = 0.40  # [m] 좌/우 후보 경로가 모두 이보다 작으면 통로 없음으로 봅니다.
NO_PATH_FRONT_THRESH = 0.30  # [m] 정면도 이보다 가까우면 회전 복구 모드 조건으로 봅니다.
ROTATE_IN_PLACE_W = 0.80  # [rad/s] 통로가 없다고 판단될 때 제자리 회전에 쓰는 각속도입니다.

STUCK_FRONT_THRESH = 0.20  # [m] 정면이 이보다 계속 가까우면 stuck 후보로 봅니다.
STUCK_TRIGGER_COUNT = 10  # stuck 판정을 내리기 전까지 가까운 상태가 반복되어야 하는 횟수입니다.
STUCK_RECOVERY_V = -0.06  # [m/s] stuck 복구에서 쓰는 후진 속도입니다.
STUCK_RESET_COUNT = 30  # stuck 복구가 너무 오래 지속될 때 카운터를 초기화하는 기준입니다.

# 라인 키핑 (좌측 흰 벽 추종)
K_LANE = 0.0  # 좌측 벽 추종 gain이며 현재는 0이라 비활성입니다.
LANE_ACTIVE_DIST = 1.5  # 정면이 이거보다 멀 때만 좌측 벽 추종을 활성화합니다.
LANE_DEADBAND_M = 0.04  # [m] 좌측 벽 거리 오차가 이 안이면 보정하지 않습니다.
W_LANE_LIMIT = 0.25  # [rad/s] 좌측 벽 추종 보정 각속도의 최대값입니다.

# 시작 시점 자동 보정 fallback
LANE_LEFT_TARGET_FALLBACK = 0.55  # 좌측 벽 거리 자동 측정 실패 시 사용할 기본 목표 거리입니다.

# 미션
MISSION_DURATION_S = 55.0  # [s] 미션 주행을 최대 몇 초 동안 할지 정합니다.


# ─────────────── 라이다 드라이버 ───────────────
class RPLidarC1:
    # RPLiDAR C1을 초기화하고 백그라운드 스레드에서 스캔을 계속 갱신하는 클래스입니다.
    def __init__(self, port, baud):
        # 지정한 포트와 baudrate로 라이다 시리얼 포트를 엽니다.
        self.ser = serial.Serial(port, baud, timeout=0.1)
        # 라이다를 리셋 명령으로 초기화하고 내부적으로 안정화될 시간을 줍니다.
        self.ser.write(bytes([0xA5, 0x40]))
        time.sleep(2.0)
        # 리셋 과정에서 남아 있을 수 있는 수신 버퍼를 비웁니다.
        self.ser.reset_input_buffer()
        # 라이다 scan 명령을 보내고 descriptor 7바이트를 읽어 버립니다.
        self.ser.write(bytes([0xA5, 0x20]))
        self.ser.read(7)
        # latest_scan을 스레드 안전하게 공유하기 위한 lock입니다.
        self.lock = threading.Lock()
        # 가장 최근 완성된 1회전 스캔 데이터를 저장할 변수입니다.
        self.latest_scan = None
        # 수신 스레드가 계속 돌아야 하는지 나타내는 플래그입니다.
        self.running = True
        # 라이다 패킷 수신 루프를 daemon thread로 생성합니다.
        self.thread = threading.Thread(target=self._loop, daemon=True)
        # 백그라운드 라이다 수신을 시작합니다.
        self.thread.start()

    def _loop(self):
        # 한 바퀴 스캔을 구성할 각도, 거리, 품질 버퍼를 준비합니다.
        buf_a, buf_d, buf_q = [], [], []
        # close()에서 running을 False로 바꿀 때까지 계속 패킷을 읽습니다.
        while self.running:
            # RPLiDAR standard scan packet은 5바이트 단위로 읽습니다.
            data = self.ser.read(5)
            # 5바이트를 온전히 못 읽으면 손상된 패킷으로 보고 다음 읽기로 넘어갑니다.
            if len(data) != 5:
                continue
            # 새 스캔 시작을 나타내는 S flag를 꺼냅니다.
            s_flag = data[0] & 0x01
            # S flag의 반전 비트인 S_inv를 꺼냅니다.
            s_inv = (data[0] & 0x02) >> 1
            # S와 S_inv가 서로 반대가 아니면 패킷이 깨졌다고 보고 버립니다.
            if s_inv != (1 - s_flag):
                continue
            # angle packet check bit가 1이 아니면 잘못된 패킷으로 보고 버립니다.
            if (data[1] & 0x01) != 1:
                continue
            # 상위 6비트 품질값을 추출합니다.
            quality = data[0] >> 2
            # 두 바이트에 걸친 각도 fixed-point 값을 degree 단위로 변환합니다.
            angle = ((data[1] >> 1) | (data[2] << 7)) / 64.0
            # 두 바이트에 걸친 거리 fixed-point 값을 mm 단위로 변환합니다.
            dist = (data[3] | (data[4] << 8)) / 4.0
            # 새 스캔 시작점이 들어왔고 이전 버퍼가 충분히 크면 한 바퀴 스캔으로 확정합니다.
            if s_flag == 1 and len(buf_a) > 50:
                # latest_scan 갱신 중 다른 스레드가 읽지 못하게 lock을 잡습니다.
                with self.lock:
                    # Python list를 numpy float32 배열로 바꿔 최신 스캔에 저장합니다.
                    self.latest_scan = (np.array(buf_a, dtype=np.float32),
                                        np.array(buf_d, dtype=np.float32),
                                        np.array(buf_q, dtype=np.float32))
                # 다음 한 바퀴를 받기 위해 임시 버퍼를 비웁니다.
                buf_a, buf_d, buf_q = [], [], []
            # 거리와 품질이 모두 유효한 점만 스캔 버퍼에 추가합니다.
            if dist > 0 and quality > 0:
                # 이번 점의 각도, 거리, 품질을 각각 버퍼에 저장합니다.
                buf_a.append(angle)
                buf_d.append(dist)
                buf_q.append(quality)

    def get_scan(self):
        # 최신 스캔을 읽는 동안 수신 스레드가 덮어쓰지 못하게 lock을 잡습니다.
        with self.lock:
            # 가장 최근 완성된 스캔을 반환합니다.
            return self.latest_scan

    def close(self):
        # 수신 스레드 루프가 종료되도록 플래그를 내립니다.
        self.running = False
        try:
            # RPLiDAR stop 명령을 보내 회전을 멈춥니다.
            self.ser.write(bytes([0xA5, 0x25]))
        except:
            # 종료 중 시리얼 예외가 나도 프로그램 종료를 계속 진행합니다.
            pass
        # stop 명령이 전송될 시간을 짧게 둔 뒤 시리얼 포트를 닫습니다.
        time.sleep(0.1)
        self.ser.close()


# ─────────────── 정면 ±FOV 거리 배열 ───────────────
def build_front_array(scan, n_bins=181):
    # 원본 라이다 스캔을 정면 -90도~+90도 거리 배열로 변환합니다.
    if scan is None:
        # 아직 스캔이 없으면 계산할 수 없으므로 None을 반환합니다.
        return None, None
    # scan tuple에서 각도, 거리, 품질 배열을 꺼내고 품질은 여기서는 사용하지 않습니다.
    a, d, _q = scan
    # 장착 각도 오프셋을 적용합니다.
    a = a + ANGLE_OFFSET_DEG
    # 거리 보정값을 더한 뒤 mm를 m로 변환합니다.
    d = (d + DIST_OFFSET_MM) / 1000.0
    # 각도를 -180도~+180도 범위로 정규화합니다.
    a = ((a + 180.0) % 360.0) - 180.0
    # 정면 FOV 안에 들어온 점만 선택합니다.
    m = np.abs(a) <= FOV_HALF_DEG
    # 선택된 각도와 거리만 남깁니다.
    a, d = a[m], d[m]
    # -90도~+90도를 n_bins개 bin으로 나눕니다.
    bins = np.linspace(-FOV_HALF_DEG, FOV_HALF_DEG, n_bins)
    # 각 bin의 최소 거리를 저장할 배열을 NaN으로 초기화합니다.
    out = np.full(n_bins, np.nan, dtype=np.float32)
    # FOV 안에 점이 하나라도 있으면 binning을 수행합니다.
    if len(a) > 0:
        # 각도를 1도 단위 bin index로 바꾸고 배열 범위 안으로 제한합니다.
        idx = np.clip((a + FOV_HALF_DEG).astype(np.int32), 0, n_bins - 1)
        # 각 라이다 점을 해당 각도 bin에 넣습니다.
        for i, di in zip(idx, d):
            # 같은 bin에 여러 점이 있으면 가장 가까운 거리만 유지합니다.
            if np.isnan(out[i]) or di < out[i]:
                out[i] = di
    # 모든 bin이 비어 있으면 유효한 정면 스캔이 없다고 봅니다.
    if np.all(np.isnan(out)):
        return None, None
    # 비어 있는 각도 bin은 멀리 비어 있다고 가정해 DISP_CLAMP로 채웁니다.
    out[np.isnan(out)] = DISP_CLAMP
    # 거리값을 0~DISP_CLAMP 범위로 제한합니다.
    out = np.clip(out, 0.0, DISP_CLAMP)
    # degree bin을 radian 각도 배열로 변환합니다.
    theta = np.deg2rad(bins)
    # 각도 배열과 거리 배열을 반환합니다.
    return theta, out


# ─────────────── corridor 침범 기반 회피각 계산 ───────────────
def min_in_angle(theta, dist, half_deg):
    # 정면 기준 ±half_deg 범위에서 가장 가까운 거리를 구합니다.
    mask = np.abs(theta) <= np.deg2rad(half_deg)
    # 해당 각도 범위가 없으면 멀리 비어 있다고 봅니다.
    if not mask.any():
        return DISP_CLAMP
    # 범위 안 최소 거리를 float으로 반환합니다.
    return float(np.min(dist[mask]))


def min_in_sector(theta, dist, deg_lo, deg_hi):
    # degree 단위 sector 경계를 radian으로 바꿉니다.
    lo = np.deg2rad(deg_lo)
    hi = np.deg2rad(deg_hi)
    # 지정한 각도 범위에 속하는 라이다 bin을 선택합니다.
    mask = (theta >= lo) & (theta <= hi)
    # sector 안에 bin이 없으면 멀리 비어 있다고 봅니다.
    if not mask.any():
        return DISP_CLAMP
    # sector 안 최소 거리를 반환합니다.
    return float(np.min(dist[mask]))


def swept_corridor_clearance(theta, dist, heading):
    # 특정 heading 방향으로 로봇 폭만큼의 가상 통로를 쓸고 지나갈 때 가장 가까운 막힘 거리를 계산합니다.
    x, y = polar_to_xy(theta, dist)
    # heading 방향 벡터 계산에 쓸 cos 값을 구합니다.
    c = math.cos(heading)
    # heading 방향 벡터 계산에 쓸 sin 값을 구합니다.
    s = math.sin(heading)
    # 각 라이다 점을 heading 방향의 전방 거리 성분으로 투영합니다.
    along = x * c + y * s
    # 각 라이다 점을 heading 방향의 좌우 거리 성분으로 투영합니다.
    lateral = -x * s + y * c
    # 로봇이 해당 heading으로 갈 때 가상 통로 안에 들어오는 점을 찾습니다.
    mask = ((along > MIN_OBS_X) &
            (along < CORRIDOR_LOOKAHEAD) &
            (np.abs(lateral) < CORRIDOR_HALF))
    # 통로 안 점이 너무 적으면 그 방향은 막히지 않은 것으로 봅니다.
    if int(mask.sum()) < BLOCK_MIN_POINTS:
        return DISP_CLAMP
    # 통로 안 점들의 가까운 쪽 10% 거리를 반환해 순간 노이즈보다 보수적으로 판단합니다.
    return float(np.percentile(along[mask], 10))


def polar_to_xy(theta, dist):
    # 라이다 극좌표(theta, dist)를 로봇 기준 직교좌표(x, y)로 바꿉니다.
    x = dist * np.cos(theta)
    # x는 로봇 전방 방향 거리입니다.
    y = dist * np.sin(theta)
    # y는 로봇 좌우 방향 거리이며 이 코드에서는 음수가 왼쪽, 양수가 오른쪽입니다.
    return x, y


def front_urgency(front):
    # 전방 장애물 거리 front를 0~1 긴급도 값으로 변환합니다.
    if front >= FRONT_URGENT_DIST:
        # 충분히 멀면 긴급도가 0입니다.
        return 0.0
    if front <= FRONT_CRITICAL_DIST:
        # 매우 가까우면 긴급도가 1입니다.
        return 1.0
    # 두 기준 사이에서는 선형 보간으로 긴급도를 계산합니다.
    return (FRONT_URGENT_DIST - front) / (FRONT_URGENT_DIST - FRONT_CRITICAL_DIST)


def find_corridor_blocker(theta, dist):
    # 현재 정면 통로 안에 들어온 가장 가까운 장애물 cluster를 찾습니다.
    x, y = polar_to_xy(theta, dist)
    # 로봇 전방, lookahead 이내, 감지 통로 폭 안에 있는 점만 장애물 후보로 선택합니다.
    mask = ((x > MIN_OBS_X) &
            (x < CORRIDOR_LOOKAHEAD) &
            (np.abs(y) < DETECT_CORRIDOR_HALF))
    # 후보 점이 너무 적으면 장애물 cluster가 없다고 판단합니다.
    if int(mask.sum()) < BLOCK_MIN_POINTS:
        return None

    # 후보 점들의 x 좌표를 따로 꺼냅니다.
    bx = x[mask]
    # 후보 점들의 y 좌표를 따로 꺼냅니다.
    by = y[mask]
    # 가장 가까운 쪽 x 위치를 10 percentile로 잡아 노이즈 한 점에 덜 민감하게 합니다.
    near_x = float(np.percentile(bx, 10))
    # near_x 주변 일정 거리 이내 점만 같은 장애물 cluster로 묶습니다.
    cluster = mask & (x <= near_x + OBSTACLE_X_WINDOW)
    # cluster 점이 너무 적으면 전체 후보를 cluster로 사용합니다.
    if int(cluster.sum()) < BLOCK_MIN_POINTS:
        cluster = mask

    # 최종 cluster의 x 좌표 배열입니다.
    cx = x[cluster]
    # 최종 cluster의 y 좌표 배열입니다.
    cy = y[cluster]
    # 장애물 대표 위치와 폭 범위, 점 개수, 가까운 전방 거리를 dict로 반환합니다.
    return {
        "x": float(np.percentile(cx, 20)),
        "y": float(np.median(cy)),
        "y_min": float(np.percentile(cy, 10)),
        "y_max": float(np.percentile(cy, 90)),
        "points": int(cluster.sum()),
        "front": float(np.min(bx)),
    }


def score_pass_heading(heading, clear, front, prev_heading, urgency):
    # 특정 후보 heading이 얼마나 좋은지 낮을수록 좋은 cost로 평가합니다.
    forward_cost = FORWARD_COST + (FORWARD_COST_URGENT - FORWARD_COST) * urgency
    # 긴급도에 따라 직전 heading 유지 비용 가중치를 보간합니다.
    prev_cost = PREV_HEADING_COST + (PREV_HEADING_COST_URGENT - PREV_HEADING_COST) * urgency
    # 긴급도에 따라 여유거리 부족 비용 가중치를 보간합니다.
    clear_cost = CLEARANCE_COST + (CLEARANCE_COST_URGENT - CLEARANCE_COST) * urgency
    # 긴급도에 따라 좌/우 방향 전환 비용 가중치를 보간합니다.
    switch_cost = SWITCH_SIGN_COST + (SWITCH_SIGN_COST_URGENT - SWITCH_SIGN_COST) * urgency

    # 큰 heading일수록 직진성 비용이 증가합니다.
    cost = forward_cost * abs(heading)
    # 직전 heading과 많이 다를수록 진동 방지 비용이 증가합니다.
    cost += prev_cost * abs(heading - prev_heading)
    # 목표 여유거리보다 clear가 부족할수록 비용이 증가합니다.
    cost += clear_cost * max(0.0, TARGET_CLEAR_DIST - clear)

    # 직전 heading의 좌/우 부호를 구하되 작은 값은 방향 없음으로 봅니다.
    prev_sign = math.copysign(1.0, prev_heading) if abs(prev_heading) > 0.12 else 0.0
    # 현재 후보 heading의 좌/우 부호를 구하되 작은 값은 방향 없음으로 봅니다.
    heading_sign = math.copysign(1.0, heading) if abs(heading) > 0.12 else 0.0
    # 직전 회피 방향과 후보 방향이 반대면 추가 비용을 줍니다.
    if prev_sign != 0.0 and heading_sign != 0.0 and heading_sign != prev_sign:
        cost += switch_cost
    # 후보 경로 여유가 너무 작으면 큰 안전 패널티를 추가합니다.
    if clear < HEADING_MIN_CLEAR:
        cost += 5.0 * (1.0 + urgency)
    # 최종 비용을 반환합니다.
    return cost


def choose_side_from_candidates(theta, dist, prev_heading, trigger_dist):
    # 여러 좌/우 heading 후보를 평가해서 가장 좋은 회피 방향과 각도를 고릅니다.
    urgency = front_urgency(trigger_dist)
    # 왼쪽 후보 중 최적값을 (heading, clear, cost) 형태로 저장합니다.
    best_left = (None, -1.0, float("inf"))
    # 오른쪽 후보 중 최적값을 (heading, clear, cost) 형태로 저장합니다.
    best_right = (None, -1.0, float("inf"))

    # 4도부터 최대 회피각까지 후보 heading 각도를 생성합니다.
    headings_deg = np.arange(HEADING_SAMPLE_DEG,
                             HEADING_MAX_DEG + 0.1,
                             HEADING_SAMPLE_DEG)
    # 각 후보 각도에 대해 왼쪽/오른쪽 양방향을 모두 평가합니다.
    for deg in headings_deg:
        # sign=-1은 왼쪽 heading, sign=+1은 오른쪽 heading입니다.
        for sign in (-1.0, 1.0):
            # degree 후보를 radian heading으로 변환합니다.
            heading = sign * np.deg2rad(deg)
            # 해당 heading으로 로봇 폭 통로를 쓸었을 때의 여유거리를 계산합니다.
            clear = swept_corridor_clearance(theta, dist, heading)
            # 후보 heading의 종합 cost를 계산합니다.
            cost = score_pass_heading(heading, clear, trigger_dist,
                                      prev_heading, urgency)
            # sign이 음수면 왼쪽 후보 best를 갱신합니다.
            if sign < 0.0:
                if cost < best_left[2]:
                    best_left = (heading, clear, cost)
            # sign이 양수면 오른쪽 후보 best를 갱신합니다.
            elif cost < best_right[2]:
                best_right = (heading, clear, cost)

    # 최종 왼쪽 후보 값을 변수로 풉니다.
    left_heading, left_clear, left_cost = best_left
    # 최종 오른쪽 후보 값을 변수로 풉니다.
    right_heading, right_clear, right_cost = best_right

    # 왼쪽 후보가 하나도 없으면 매우 나쁜 기본 후보로 채웁니다.
    if left_heading is None:
        left_heading, left_clear, left_cost = -np.deg2rad(HEADING_MAX_DEG), 0.0, float("inf")
    # 오른쪽 후보가 하나도 없으면 매우 나쁜 기본 후보로 채웁니다.
    if right_heading is None:
        right_heading, right_clear, right_cost = np.deg2rad(HEADING_MAX_DEG), 0.0, float("inf")

    # 긴급하고 좌/우 clear 차이가 충분히 크면 비용보다 더 열린 방향을 우선합니다.
    if urgency > 0.35 and abs(left_clear - right_clear) > CLEAR_DECISIVE_MARGIN:
        # 왼쪽 clear가 더 크면 왼쪽 후보를 선택합니다.
        if left_clear > right_clear:
            return left_heading, left_clear, left_clear, right_clear, left_cost, right_cost, urgency
        # 오른쪽 clear가 더 크면 오른쪽 후보를 선택합니다.
        return right_heading, right_clear, left_clear, right_clear, left_cost, right_cost, urgency

    # 비용이 더 낮은 쪽을 최종 회피 후보로 선택합니다.
    if left_cost <= right_cost:
        return left_heading, left_clear, left_clear, right_clear, left_cost, right_cost, urgency
    # 오른쪽 비용이 더 낮으면 오른쪽 후보를 선택합니다.
    return right_heading, right_clear, left_clear, right_clear, left_cost, right_cost, urgency


def choose_heading_corridor(theta, dist, prev_heading):
    # 현재 스캔에서 최종 목표 heading, front 거리, 경로 여유 등을 계산합니다.
    blocker = find_corridor_blocker(theta, dist)
    # 정면 heading 0도로 갔을 때 로봇 통로가 얼마나 앞에서 막히는지 계산합니다.
    straight_clear = swept_corridor_clearance(theta, dist, 0.0)
    # 정면 ±FRONT_HALF_DEG 안의 가장 가까운 거리를 계산합니다.
    front = min_in_angle(theta, dist, FRONT_HALF_DEG)
    # 회피 긴급도 판단에 사용할 대표 거리를 front와 straight_clear 중 더 작은 값으로 둡니다.
    trigger_dist = min(front, straight_clear)
    # 명시적인 blocker가 있으면 blocker의 가장 가까운 거리도 trigger에 반영합니다.
    if blocker is not None:
        trigger_dist = min(trigger_dist, blocker["front"])

    # 직진 통로가 충분히 비어 있고 정면도 멀면 heading 0으로 직진을 선택합니다.
    if straight_clear >= TARGET_CLEAR_DIST and front >= FRONT_URGENT_DIST:
        return 0.0, front, straight_clear, blocker, 0.0, 0.0, 0.0, 0.0, 0.0

    # 직진이 충분히 안전하지 않으면 후보 heading들 중 최적 회피 방향을 고릅니다.
    heading, clear, left_clear, right_clear, left_cost, right_cost, urgency = (
        choose_side_from_candidates(theta, dist, prev_heading, trigger_dist)
    )
    # 메인 루프에서 로그와 제어에 필요한 값들을 모두 반환합니다.
    return heading, trigger_dist, clear, blocker, left_clear, right_clear, left_cost, right_cost, urgency


def side_guard_w(theta, dist):
    # 좌우 측면이 너무 가까운 경우 반대쪽으로 살짝 밀어주는 보정 각속도를 계산합니다.
    left_near = min_in_sector(theta, dist, -60.0, -15.0)
    # 오른쪽 측면 sector에서 가장 가까운 거리를 구합니다.
    right_near = min_in_sector(theta, dist, 15.0, 60.0)
    # 왼쪽이 기준거리보다 가까운 만큼 push 값을 만듭니다.
    left_push = max(0.0, SIDE_GUARD_DIST - left_near)
    # 오른쪽이 기준거리보다 가까운 만큼 push 값을 만듭니다.
    right_push = max(0.0, SIDE_GUARD_DIST - right_near)
    # 오른쪽이 가까우면 양수, 왼쪽이 가까우면 음수 방향으로 보정하도록 계산합니다.
    w = SIDE_GUARD_GAIN * (right_push - left_push)
    # 보정 각속도를 제한하고 측면 거리도 함께 반환합니다.
    return float(np.clip(w, -SIDE_GUARD_W_MAX, SIDE_GUARD_W_MAX)), left_near, right_near


def apply_deadband(value, deadband):
    # 값의 절댓값이 deadband보다 작으면 0으로 만들어 작은 떨림을 제거합니다.
    return 0.0 if abs(value) < deadband else value


def apply_min_avoid_turn(w_target, heading, urgency):
    # 급한 회피 상황에서 각속도가 너무 작으면 최소 회전량을 보장합니다.
    if urgency <= 0.0 or abs(heading) <= np.deg2rad(HEADING_DEADBAND_DEG):
        # 급하지 않거나 heading이 너무 작으면 원래 목표 각속도를 그대로 씁니다.
        return w_target
    # 긴급도에 비례해 필요한 최소 각속도를 계산합니다.
    min_w = MIN_AVOID_W_URGENT * urgency
    # heading 부호와 실제 w 부호 관계에 맞춰 회전 방향을 정합니다.
    turn_sign = -math.copysign(1.0, heading)
    # 현재 목표 각속도가 필요한 방향으로 min_w보다 작으면 보강합니다.
    if w_target * turn_sign < min_w:
        return turn_sign * min_w
    # 이미 충분히 회전하고 있으면 원래 값을 유지합니다.
    return w_target


def smooth_steering(prev_w, target_w, urgency=0.0):
    # 목표 각속도 target_w를 필터링하고 변화율 제한을 걸어 부드럽게 만듭니다.
    alpha = W_SMOOTH_ALPHA + (W_URGENT_SMOOTH_ALPHA - W_SMOOTH_ALPHA) * urgency
    # 긴급도에 따라 한 루프에서 허용할 각속도 변화량을 보간합니다.
    rate_limit = W_RATE_LIMIT_STEP + (W_URGENT_RATE_LIMIT_STEP - W_RATE_LIMIT_STEP) * urgency
    # 1차 저역통과 필터로 목표 각속도를 부드럽게 따라갑니다.
    filtered = prev_w + alpha * (target_w - prev_w)
    # 필터 결과가 직전 값에서 너무 멀리 뛰지 않도록 변화량을 제한합니다.
    step = float(np.clip(filtered - prev_w,
                         -rate_limit,
                         rate_limit))
    # 제한된 변화량을 직전 각속도에 더해 최종 각속도를 반환합니다.
    return prev_w + step


def cruise_velocity(front, clear):
    # 전방거리와 경로 여유거리 기반으로 전진 속도를 스케일링합니다.
    s_f = float(np.clip((front - V_SCALE_FRONT_NEAR) /
                        (V_SCALE_FRONT_FAR - V_SCALE_FRONT_NEAR),
                        0.0, 1.0))
    # 경로 clear 기반 속도 스케일을 0~1로 계산합니다.
    s_c = float(np.clip((clear - V_SCALE_CLEAR_NEAR) /
                        (V_SCALE_CLEAR_FAR - V_SCALE_CLEAR_NEAR),
                        0.0, 1.0))
    # 두 스케일 중 더 보수적인 값을 V_CRUISE에 곱해 반환합니다.
    return V_CRUISE * min(s_f, s_c)


# ─────────────── 좌측(흰 벽) 거리: -90° 영역 ───────────────
def left_wall_distance(scan):
    """-90° 근처 ±15° 영역의 median [m]."""
    # 좌측 벽 거리 추종/초기 보정에 사용할 -90도 근처 거리 median을 계산합니다.
    if scan is None:
        # 스캔이 없으면 거리도 계산할 수 없습니다.
        return None
    # scan tuple에서 각도와 거리 배열을 꺼내고 품질은 사용하지 않습니다.
    a, d, _ = scan
    # 장착 각도 오프셋을 적용합니다.
    a = a + ANGLE_OFFSET_DEG
    # 각도를 -180도~+180도 범위로 정규화합니다.
    a = ((a + 180.0) % 360.0) - 180.0
    # 좌측 -90도 주변, 거리 50~4000mm 사이의 점만 선택합니다.
    mask = (a > -105) & (a < -75) & (d > 50) & (d < 4000)
    # 유효한 점이 너무 적으면 측정 실패로 봅니다.
    if mask.sum() < 5:
        return None
    # 선택된 거리의 median을 m 단위로 반환합니다.
    return float(np.median(d[mask])) / 1000.0


# ─────────────── 메인 ───────────────
def main():
    # RPLiDAR 객체를 생성하고 백그라운드 스캔 수신을 시작합니다.
    lidar = RPLidarC1(LIDAR_PORT, LIDAR_BAUD)
    # Arduino UART 포트를 엽니다.
    ardu = serial.Serial(ARDU_PORT, ARDU_BAUD, timeout=0.1)
    # 라이다/시리얼이 안정화될 시간을 둡니다.
    print("[INFO] 워밍업 2초...")
    time.sleep(2.0)

    # ─── 시작 시점 자동 보정 ───
    # 시작 위치에서 좌측 벽 거리를 1초 동안 측정해 lane target으로 삼습니다.
    print("[CALIB] 좌측 벽 거리 측정 (1초)...")
    # 좌측 벽 거리 샘플을 모을 list입니다.
    samples = []
    # 1초 뒤의 종료 시각을 계산합니다.
    t_end = time.time() + 1.0
    # 현재 시각이 종료 시각보다 작을 동안 샘플링합니다.
    while time.time() < t_end:
        # 가장 최근 라이다 스캔을 가져옵니다.
        s = lidar.get_scan()
        # 스캔이 있으면 좌측 벽 거리 계산을 시도합니다.
        if s is not None:
            # -90도 주변 median 거리입니다.
            d = left_wall_distance(s)
            # 유효한 거리면 샘플 list에 추가합니다.
            if d is not None:
                samples.append(d)
        # 20Hz 정도로 샘플을 모읍니다.
        time.sleep(0.05)
    # 샘플이 충분하면 median 값을 목표 좌측 벽 거리로 사용합니다.
    if len(samples) >= 5:
        # 샘플 median으로 lane target을 정합니다.
        lane_target = float(np.median(samples))
        # 측정 결과와 표준편차를 로그로 출력합니다.
        print(f"[CALIB] LANE_LEFT_TARGET = {lane_target:.3f}m  "
              f"(n={len(samples)}, σ={np.std(samples)*1000:.1f}mm)")
    else:
        # 샘플이 부족하면 fallback 값을 사용합니다.
        lane_target = LANE_LEFT_TARGET_FALLBACK
        # fallback 사용 사실을 로그로 출력합니다.
        print(f"[CALIB] 측정 실패, fallback {lane_target}m 사용")

    def send_vw(v, w):
        # Arduino가 파싱하는 V선속도,각속도 형식으로 명령을 보냅니다.
        ardu.write(f"V{v:.3f},{w:.3f}\n".encode())

    def stop():
        # Arduino에 정지 명령 S를 보냅니다.
        ardu.write(b"S\n")

    # 실제 주행 시작 전 3초 대기 안내를 출력합니다.
    print("[INFO] 3초 후 주행 시작...")
    # 로봇을 내려놓거나 준비할 시간을 줍니다.
    time.sleep(3.0)
    # 주행 시작 로그입니다.
    print("[GO]")
    # 미션 시작 기준 시각입니다.
    t0 = time.time()
    # 마지막으로 유효 스캔을 받은 시각입니다.
    last_scan_ok = 0.0
    # 마지막으로 보낸 선속도/각속도 명령입니다.
    last_v, last_w = 0.0, 0.0
    # 마지막 로그 출력 시각입니다.
    last_log = 0.0
    # 직전 목표 heading입니다.
    prev_heading = 0.0
    # 회피 중 누적 회전량을 저장하는 값입니다.
    turn_balance = 0.0
    # 직전 명령 갱신 시각입니다.
    last_cmd_time = time.time()
    # 정면이 매우 가까운 상태가 연속된 횟수입니다.
    stuck_counter = 0
    try:
        # 미션 시간 종료 또는 KeyboardInterrupt 전까지 제어 루프를 반복합니다.
        while True:
            # 미션 시작 후 경과 시간을 계산합니다.
            t = time.time() - t0
            # 제한 시간을 넘으면 주행을 종료합니다.
            if t > MISSION_DURATION_S:
                print("[INFO] 시간 초과 정지")
                break

            # 가장 최근 라이다 스캔을 가져옵니다.
            scan = lidar.get_scan()
            # 아직 스캔이 없으면 짧은 시간은 직전 명령을 유지하고 길어지면 정지합니다.
            if scan is None:
                if time.time() - last_scan_ok <= SCAN_HOLD_S:
                    # 짧은 스캔 누락이므로 마지막 명령을 다시 보냅니다.
                    send_vw(last_v, last_w)
                else:
                    # 스캔이 오래 없으면 안전을 위해 정지 명령을 보냅니다.
                    send_vw(0, 0)
                # 루프 주기를 맞춘 뒤 다음 반복으로 넘어갑니다.
                time.sleep(LOOP_DT_S)
                continue

            # 원본 스캔을 정면 각도/거리 배열로 변환합니다.
            theta, dist = build_front_array(scan)
            # 변환에 실패하면 scan None과 같은 방식으로 처리합니다.
            if theta is None:
                if time.time() - last_scan_ok <= SCAN_HOLD_S:
                    # 짧은 변환 실패이므로 마지막 명령을 유지합니다.
                    send_vw(last_v, last_w)
                else:
                    # 변환 실패가 오래가면 정지합니다.
                    send_vw(0, 0)
                # 루프 주기를 맞춘 뒤 다음 반복으로 넘어갑니다.
                time.sleep(LOOP_DT_S)
                continue
            # 유효한 스캔을 받은 시각을 갱신합니다.
            last_scan_ok = time.time()

            # 현재 라이다 스캔에서 목표 heading과 경로 상태를 계산합니다.
            heading, front_dist, path_clear, blocker, left_clear, right_clear, left_cost, right_cost, urgency = (
                choose_heading_corridor(theta, dist, prev_heading)
            )
            # 좌우 측면 근접 보호용 각속도 보정과 측면 거리를 계산합니다.
            w_guard, left_near, right_near = side_guard_w(theta, dist)

            # 정면이 매우 가까우면 stuck_counter를 증가시키고 아니면 0으로 리셋합니다.
            stuck_counter = stuck_counter + 1 if front_dist < STUCK_FRONT_THRESH else 0
            # 좌/우 후보 경로가 모두 좁으면 좋은 경로가 없다고 판단합니다.
            no_good_path = max(left_clear, right_clear) < NO_PATH_CLEAR_THRESH
            # 정면이 회전 복구 기준보다 가까운지 판단합니다.
            critical_front = front_dist < NO_PATH_FRONT_THRESH

            # 정면 근접 상태가 일정 횟수 이상 반복되면 stuck 복구 모드로 들어갑니다.
            if stuck_counter > STUCK_TRIGGER_COUNT:
                # 로그에 표시할 현재 모드를 STUCK으로 둡니다.
                current_mode = "STUCK"
                # stuck 복구를 위해 설정된 후진 속도를 사용합니다.
                v = STUCK_RECOVERY_V
                # 좌/우 clear 차이에 따라 회전 방향을 고릅니다.
                target_w = math.copysign(W_MAX, left_clear - right_clear)
                # 급한 상황으로 보고 각속도를 빠르게 목표로 보냅니다.
                w = smooth_steering(last_w, target_w, 1.0)
                # stuck 복구 카운터가 너무 커지면 다시 0부터 판단하게 초기화합니다.
                if stuck_counter > STUCK_RESET_COUNT:
                    stuck_counter = 0
            # 좋은 후보 경로가 없고 정면도 가까우면 제자리 회전 모드로 들어갑니다.
            elif no_good_path and critical_front:
                # 로그에 표시할 현재 모드를 ROTATE로 둡니다.
                current_mode = "ROTATE"
                # 제자리 회전을 위해 선속도를 0으로 둡니다.
                v = 0.0
                # 좌/우 clear 차이에 따라 제자리 회전 방향을 고릅니다.
                target_w = math.copysign(ROTATE_IN_PLACE_W, left_clear - right_clear)
                # 급한 상황으로 보고 각속도를 빠르게 목표로 보냅니다.
                w = smooth_steering(last_w, target_w, 1.0)
            else:
                # 일반 주행/회피 모드에서 lane 보정 각속도를 0으로 초기화합니다.
                w_lane = 0.0
                # 회피 heading이 없고 정면이 멀 때만 좌측 벽 추종을 시도합니다.
                if heading == 0.0 and front_dist > LANE_ACTIVE_DIST:
                    # 현재 좌측 벽 거리를 측정합니다.
                    lwd = left_wall_distance(scan)
                    # 좌측 벽 거리 측정이 성공했을 때만 보정을 계산합니다.
                    if lwd is not None:
                        # 목표 좌측 거리와 현재 좌측 거리의 오차입니다.
                        err = lane_target - lwd
                        # deadband보다 큰 오차만 보정합니다.
                        if abs(err) > LANE_DEADBAND_M:
                            # deadband만큼을 빼서 작은 오차에 둔감하게 만듭니다.
                            lane_err = err - math.copysign(LANE_DEADBAND_M, err)
                            # lane_err를 각속도 보정으로 바꾸고 최대값을 제한합니다.
                            w_lane = float(np.clip(-K_LANE * lane_err,
                                                   -W_LANE_LIMIT,
                                                   W_LANE_LIMIT))

                # 목표 heading을 실제 각속도 명령으로 변환합니다.
                w_heading = -K_HEADING * heading
                # heading 보정, 측면 보호, lane 보정을 합산한 뒤 최대 각속도를 제한합니다.
                w_target = float(np.clip(w_heading + w_guard + w_lane,
                                         -W_MAX,
                                         W_MAX))
                # 급한 회피에서 각속도가 너무 작으면 최소 회전 각속도를 보장합니다.
                w_target = apply_min_avoid_turn(w_target, heading, urgency)
                # 작은 각속도 명령을 0으로 만들어 떨림을 줄입니다.
                w_target = apply_deadband(w_target, W_DEADBAND)
                # 각속도 명령을 필터링해 부드럽게 만듭니다.
                w = smooth_steering(last_w, w_target, urgency)
                # 전방거리와 경로 여유거리 기반으로 선속도를 계산합니다.
                v = cruise_velocity(front_dist, path_clear)
                # heading이 deadband보다 크면 회피 모드로 표시합니다.
                if abs(heading) > np.deg2rad(HEADING_DEADBAND_DEG):
                    current_mode = "AVOID"
                # heading은 작지만 측면 보호가 작동 중이면 GUARD 모드로 표시합니다.
                elif abs(w_guard) > W_DEADBAND:
                    current_mode = "GUARD"
                # 위 조건이 모두 아니면 직진 모드로 표시합니다.
                else:
                    current_mode = "STRAIGHT"

            # 계산된 선속도/각속도를 Arduino로 전송합니다.
            send_vw(v, w)
            # 현재 시각을 저장합니다.
            now = time.time()
            # 누적 회전량 계산용 dt를 0~0.20초로 제한합니다.
            dt_cmd = max(0.0, min(0.20, now - last_cmd_time))
            # 다음 루프를 위해 마지막 명령 시각을 갱신합니다.
            last_cmd_time = now
            # 목표 heading이 충분히 크면 현재 회피 중이라고 판단합니다.
            is_avoiding = abs(heading) > np.deg2rad(HEADING_DEADBAND_DEG)
            # 회피 중이면 실제 w를 시간 적분해 turn_balance를 갱신합니다.
            if is_avoiding:
                turn_balance = float(np.clip(turn_balance - w * dt_cmd,
                                             -TURN_BALANCE_CLAMP,
                                             TURN_BALANCE_CLAMP))
            else:
                # 회피가 아니면 turn_balance를 서서히 0으로 감쇠시킵니다.
                balance_decay = max(0.0, 1.0 - TURN_BALANCE_DECAY_PER_S * dt_cmd)
                # 감쇠 계수를 곱해 누적 회전량을 줄입니다.
                turn_balance *= balance_decay
            # 다음 루프에서 스캔 누락 시 재전송할 마지막 명령을 저장합니다.
            last_v, last_w = v, w
            # 다음 루프의 방향 전환 비용 계산에 쓸 직전 heading을 저장합니다.
            prev_heading = heading
            # 로그는 너무 많이 찍히지 않도록 약 0.25초마다 출력합니다.
            if time.time() - last_log > 0.25:
                # blocker가 있으면 대표 x를 로그에 넣고 없으면 0으로 둡니다.
                obs_x = blocker["x"] if blocker is not None else 0.0
                # blocker가 있으면 대표 y를 로그에 넣고 없으면 0으로 둡니다.
                obs_y = blocker["y"] if blocker is not None else 0.0
                # blocker가 있으면 점 개수를 로그에 넣고 없으면 0으로 둡니다.
                obs_n = blocker["points"] if blocker is not None else 0
                # 현재 제어 상태와 주요 판단값을 한 줄 로그로 출력합니다.
                print(f"[RUN] {current_mode} v={v:.2f} w={w:.2f} "
                      f"head={math.degrees(heading):.1f} "
                      f"front={front_dist:.2f} clear={path_clear:.2f} "
                      f"obs=({obs_x:.2f},{obs_y:.2f},n={obs_n}) "
                      f"passL={left_clear:.2f} passR={right_clear:.2f} "
                      f"sideL={left_near:.2f} sideR={right_near:.2f} "
                      f"urg={urgency:.2f} cL={left_cost:.2f} cR={right_cost:.2f} "
                      f"bal={math.degrees(turn_balance):.1f} stk={stuck_counter}")
                # 마지막 로그 출력 시각을 갱신합니다.
                last_log = time.time()
            # 목표 제어 주기에 맞춰 잠시 대기합니다.
            time.sleep(LOOP_DT_S)

    except KeyboardInterrupt:
        # Ctrl+C가 들어오면 finally에서 정리하도록 여기서는 조용히 빠져나갑니다.
        pass
    finally:
        # 어떤 종료 경로에서도 Arduino에 정지 명령을 보냅니다.
        stop()
        # 정지 명령이 전송될 시간을 짧게 둡니다.
        time.sleep(0.2)
        # Arduino 시리얼 포트를 닫습니다.
        ardu.close()
        # 라이다를 멈추고 시리얼 포트를 닫습니다.
        lidar.close()
        # 종료 로그를 출력합니다.
        print("[INFO] 종료")


if __name__ == "__main__":
    # 이 파일을 직접 실행했을 때만 main()을 호출합니다.
    main()
