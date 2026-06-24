# -*- coding: utf-8 -*-
"""
drive_sim — project3.py / project3.ino 를 '그대로' 돌려보는 2D 시뮬레이터.

  실행:  python3 sim.py
  의존:  pygame, numpy  (project3.py 의 결정 코드는 실제로 import 되어 호출됨)

  조작
    좌측 도구 선택 후 아레나 클릭으로 배치:
      Obstacle  : 장애물(직사각형) 배치. 클릭=중심, 드래그=회전각
      Red/Yellow/Blue : 색종이(영역) 배치
      Robot     : 클릭=위치, 드래그=초기 방향
      Erase     : 클릭 근처 객체 삭제
    버튼: Start(주행 시작) / Pause / Reset / Clear / Save / Load
    우측 슬라이더로 실제 코드의 파라미터를 조정(휠로 패널 스크롤).
    project3.py 나 project3.ino 를 저장하면 자동으로 다시 로드됨.
"""

import copy
import json
import math
import os
import sys
from datetime import datetime

import numpy as np
import pygame

from code_bridge import CodeBridge, recompute_derived, PROJECT3_DIR
from arduino_model import ArduinoSim
from world import (
    World, Obstacle, Patch, ARENA_M, TEST_ZONE_M, TEST_ZONE_X, TEST_ZONE_Y, ORDER,
    DEFAULT_OBSTACLE_Z_H, DEFAULT_ROBOT_BODY_LENGTH_M, DEFAULT_ROBOT_BODY_WIDTH_M,
    polygons_intersect,
)
import sensors

BASE_MARGIN = 20
BASE_PANEL_W = 430
BASE_ARENA_PX = 700
BASE_FONT_SIZE = 14
BASE_BIGFONT_SIZE = 16
BASE_UI_SCALE = 1.25
MIN_UI_SCALE = 0.95
MAX_UI_SCALE = 2.20
MIN_ARENA_PX = 360
MIN_VIEW_ZOOM = 0.5
MAX_VIEW_ZOOM = 8.0

UI_SCALE = BASE_UI_SCALE
MARGIN = int(round(BASE_MARGIN * UI_SCALE))
PANEL_W = int(round(BASE_PANEL_W * UI_SCALE))
ARENA_PX = BASE_ARENA_PX
PANEL_X = MARGIN + ARENA_PX + MARGIN
WIN_W = PANEL_X + PANEL_W + MARGIN
WIN_H = MARGIN + ARENA_PX + MARGIN
SCALE = ARENA_PX / ARENA_M
SIM_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(SIM_DIR, "data")
LAYOUT_PATH = os.path.join(DATA_DIR, "layout.json")
LEGACY_LAYOUT_PATH = os.path.join(SIM_DIR, "layout.json")

BG = (24, 26, 32)
ARENA_BG = (40, 43, 52)
TEST_ZONE_BG = (50, 62, 54)
TEST_ZONE_EDGE = (110, 180, 130)
KEEPIN_EDGE = (230, 180, 90)
GRID = (54, 58, 70)
WALL = (90, 96, 112)
TXT = (220, 224, 232)
TXT_DIM = (150, 156, 168)
OBST = (30, 32, 38)
OBST_EDGE = (120, 126, 140)
ROBOT_C = (90, 200, 160)
HEAD_C = (250, 250, 255)
LIDAR_C = (255, 140, 90)
TRAJ_C = (120, 220, 255)
FOV_C = (90, 180, 200)
CAM_AREA_C = (90, 180, 200, 34)
BEAR_C = (230, 110, 220)
PATCH_COL = {"RED": (210, 70, 70), "YELLOW": (220, 200, 70), "BLUE": (80, 120, 220)}
BTN = (60, 64, 78)
BTN_ON = (70, 130, 110)
BTN_HL = (84, 90, 108)

PARAM_TIPS = {
    "CRUISE_V": "기본 전진 속도입니다.",
    "MAX_W": "허용하는 최대 회전 속도입니다.",
    "W_RATE": "회전 변화량을 부드럽게 제한합니다.",
    "W_CLEAR": "장애물 여유거리 점수의 비중입니다.",
    "W_GOAL": "목표 방향 정렬 점수의 비중입니다.",
    "W_SPEED": "안전한 후보 중 빠른 전진을 선호하는 비중입니다.",
    "W_TURN": "큰 회전 명령을 줄이는 비중입니다.",
    "W_SMOOTH": "직전 회전속도와 비슷한 명령을 선호하는 비중입니다.",
    "HORIZON_T": "DWA가 앞으로 예측해보는 시간입니다.",
    "COLLISION_DIST": "이 거리 안의 장애물은 충돌 위험으로 봅니다.",
    "SLOW_DIST": "장애물 근처에서 감속을 시작하는 거리입니다.",
    "ROBOT_RADIUS": "project3.py의 DWA 충돌 예측에 쓰는 원형 근사 반지름입니다.",
    "KEEPIN_SIZE_M": "로봇 판단 로직이 벗어나지 않으려는 정사각형 영역 크기입니다.",
    "KEEPIN_MARGIN_M": "keep-in 경계에서 로봇 중심이 남겨야 하는 여유입니다.",
    "MAX_RANGE_M": "project3.py가 사용할 라이다 최대 거리입니다.",
    "V_MIN_RATIO": "막혔을 때 유지할 최소 전진 속도 비율입니다.",
    "V_SET_MIN": "다중 속도 DWA가 평가할 최저 전진 속도입니다.",
    "CLEAR_CAP": "여유거리 점수의 상한값입니다.",
    "GAP_BEARING_MAX": "막힌 목표를 우회할 때 탐색할 좌우 최대 방위각입니다.",
    "GAP_BEARING_STEP": "회피 gap 후보 방위각의 샘플 간격입니다.",
    "GAP_WINDOW_RAD": "gap 후보 주변 장애물을 확인하는 각도 폭입니다.",
    "GAP_MIN_CLEAR_MARGIN": "gap 후보가 충돌거리보다 더 가져야 하는 최소 여유입니다.",
    "GAP_PASS_LOOKAHEAD_M": "gap이 로봇 폭으로 실제 통과 가능한지 확인하는 전방 거리입니다.",
    "GAP_PASS_SIDE_MARGIN_M": "gap 통과성 검사에서 로봇 좌우 폭에 더하는 여유입니다.",
    "GAP_W_PASS": "gap 통과 폭 여유 점수의 비중입니다.",
    "AVOID_TRIGGER_MARGIN": "직접 경로 여유가 이 값보다 작으면 회피를 검토합니다.",
    "AVOID_RELEASE_MARGIN": "직접 경로가 이만큼 넓어지면 회피 상태를 풉니다.",
    "AVOID_LOST_MEMORY_S": "회피 중 색을 잠깐 잃어도 gap 방향을 유지하는 시간입니다.",
    "ESCAPE_ROTATE_MARGIN": "안전 여유 안에서 제자리 회전 탈출을 허용할 몸체 추가 여유입니다.",
    "ESCAPE_CREEP_MAX_V": "안전 여유 안에서 탈출용으로만 허용하는 최대 저속 전진입니다.",
    "ESCAPE_REVERSE_V": "안전 여유 안쪽에서만 쓰는 후진 탈출 속도 크기입니다.",
    "ESCAPE_REVERSE_MIN_GAIN": "후진 탈출 후보가 현재보다 좋아져야 하는 최소 여유 증가량입니다.",
    "HFOV_DEG": "색상 중심 오차를 방위각으로 바꾸는 기준 시야각입니다.",
    "MIN_AREA": "실제 카메라 색상 인식에서 점 잡음을 거르는 최소 픽셀 면적입니다.",
    "APPROACH_CY": "저속 정렬 접근을 시작하는 카메라 세로 위치 기준입니다.",
    "APPROACH_ALIGN_MAX": "commit 전에 허용하는 색상 중심 방위각 오차입니다.",
    "APPROACH_STABLE_S": "정렬 상태가 유지되어야 하는 시간입니다.",
    "CLOSE_LOST_HOLD_CY": "가까이 본 뒤 색을 잃으면 도착으로 보고 멈추는 기준입니다.",
    "CREEP_STEER_GAIN": "색을 잃은 뒤 마지막 bearing을 따라가는 조향 게인입니다.",
    "CREEP_MAX_W": "CREEP 중 조향 회전속도 상한입니다.",
    "DWELL_S": "도착 판정 후 다음 색으로 넘어가기 전 정지 유지 시간입니다.",
    "SEARCH_W": "색상 미검출 시 좌우 스윕 회전 속도입니다.",
    "SEARCH_SWEEP_ANGLE_DEG": "색상 미검출 시 현재 자세 기준 좌우로 훑는 각도입니다.",
    "SEARCH_SETTLE_RAD": "스윕 목표 각도에 이만큼 가까워지면 반대 방향으로 바꿉니다.",
    "SEARCH_SCAN_EDGE_HITS": "이 횟수만큼 좌우 끝을 훑고도 못 찾으면 전진 탐색합니다.",
    "SEARCH_DRIVE_V": "스캔 후 못 찾았을 때 전진 탐색 속도입니다.",
    "SEARCH_DRIVE_S": "스캔 후 못 찾았을 때 전진 탐색하는 시간입니다.",
    "EXPLORE_SPACING_M": "전역 탐색 waypoint 사이 간격입니다.",
    "EXPLORE_REACH_DIST_M": "전역 탐색 waypoint 도착 판정 거리입니다.",
    "EXPLORE_SCAN_EDGE_HITS": "waypoint 도착 후 좌우 스캔을 반복하는 횟수입니다.",
    "EXPLORE_MAX_V": "전역 탐색 중 최대 전진 속도입니다.",
    "EXPLORE_SLOW_DIST_M": "waypoint 근처에서 감속하는 거리입니다.",
    "EXPLORE_BLOCKED_S": "waypoint가 막혔다고 보고 다음 목표로 넘기는 시간입니다.",
    "EXPLORE_STUCK_S": "진전이 없을 때 다른 waypoint로 전환하는 시간입니다.",
    "WRONG_COLOR_AVOID_RADIUS_M": "현재 목표가 아닌 색의 기억 위치 주변을 후순위로 미는 반경입니다.",
    "WRONG_COLOR_SCORE_PENALTY": "현재 목표가 아닌 색 주변 waypoint에 주는 패널티입니다.",
    "TARGET_MEMORY_REACHED_DIST_M": "기억한 목표 색 위치에 도착했다고 보는 거리입니다.",
    "BLIND_CREEP_DIST_M": "카메라가 색을 잃은 뒤 더 전진할 거리입니다.",
    "CENTER_TOL_M": "색상 중심과 로봇 중심이 이 거리 안이면 도착으로 봅니다.",
    "CENTER_MAX_V": "색상 중심 정렬 단계의 최대 전진 속도입니다.",
    "CENTER_SLOW_RADIUS_M": "색상 중심에 가까워질수록 감속을 시작하는 거리입니다.",
    "CENTER_GOAL_ALPHA": "색상 중심 좌표 추정값을 부드럽게 갱신하는 비율입니다.",
    "CENTER_LOST_MEMORY_S": "색을 잠깐 잃어도 마지막 중심 좌표로 접근하는 시간입니다.",
    "CENTER_MAX_BEARING": "중심 정렬 목표 방위각의 최대 크기입니다.",
    "PRACTICE10_FX_640": "practice10.py의 640x480 기준 fx 값입니다.",
    "PRACTICE10_FY_480": "practice10.py의 640x480 기준 fy 값입니다.",
    "PRACTICE10_CX_640": "practice10.py의 640x480 기준 cx 값입니다.",
    "PRACTICE10_CY_480": "practice10.py의 640x480 기준 cy 값입니다.",
    "CAMERA_RECT_WIDTH_M": "project3.py가 실제 카메라 픽셀을 바닥 좌표로 바꿀 때 쓰는 시야 폭입니다.",
    "CAMERA_RECT_DEPTH_M": "project3.py가 실제 카메라 픽셀을 바닥 좌표로 바꿀 때 쓰는 시야 깊이입니다.",
    "CAMERA_REAR_BLIND_M": "project3.py 기준 로봇 뒤쪽 끝에서 보이지 않는 거리입니다.",
    "wheel tau": "바퀴 속도 응답이 목표에 따라가는 시간입니다.",
    "wheel accel": "바퀴 각속도의 최대 변화율입니다.",
    "sim speed": "시뮬레이션 시간 배속입니다.",
    "lidar noise": "라이다 거리값에 넣는 임의 오차입니다.",
    "n_rays": "합성 라이다 광선 개수입니다.",
    "robot height": "로봇 본체의 수직 높이입니다.",
    "robot length": "로봇 본체의 전후 길이입니다.",
    "robot width": "로봇 본체의 좌우 폭입니다.",
    "camera lift": "로봇 본체 위로 카메라가 더 올라간 높이입니다.",
    "cam forward ofs": "로봇 중심 기준 카메라 전방 오프셋입니다. 음수면 뒤쪽입니다.",
    "cam left ofs": "로봇 중심 기준 카메라 좌측 오프셋입니다.",
    "cam rear blind": "로봇 뒤쪽 끝에서 전방으로 보이지 않는 거리입니다.",
    "cam width": "카메라가 내려다보는 직사각형 영역의 좌우 폭입니다.",
    "cam depth": "카메라가 내려다보는 직사각형 영역의 전후 길이입니다.",
    "obstacle w": "장애물 발자국의 가로 폭입니다.",
    "obstacle depth": "장애물 발자국의 세로 깊이입니다.",
    "obstacle z": "카메라 가림 판정에 쓰는 장애물 높이입니다.",
    "patch size": "색상 목표 영역 한 변의 길이입니다.",
}

FONT_REGULAR_PATHS = [
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
]
FONT_BOLD_PATHS = [
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
    "/usr/share/fonts/truetype/noto/NotoSansCJK-Bold.ttc",
]
FONT_NAMES = [
    "Noto Sans Mono CJK KR",
    "Noto Sans CJK KR",
    "Noto Sans CJK",
    "Noto Sans KR",
    "NanumGothic",
    "UnDotum",
    "Malgun Gothic",
    "AppleGothic",
    "DejaVu Sans",
]


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def ui(value):
    return max(1, int(round(value * UI_SCALE)))


def json_default(obj):
    if isinstance(obj, np.generic):
        return obj.item()
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    raise TypeError("Object of type {} is not JSON serializable".format(type(obj).__name__))


def load_ui_font(size, bold=False):
    for path in (FONT_BOLD_PATHS if bold else FONT_REGULAR_PATHS):
        if os.path.exists(path):
            try:
                return pygame.font.Font(path, size)
            except pygame.error:
                pass
    for name in FONT_NAMES:
        path = pygame.font.match_font(name, bold=bold)
        if path:
            try:
                return pygame.font.Font(path, size)
            except pygame.error:
                pass
    return pygame.font.SysFont("dejavusans,sans", size, bold=bold)


def make_fonts():
    font_size = max(10, int(round(BASE_FONT_SIZE * UI_SCALE)))
    big_size = max(12, int(round(BASE_BIGFONT_SIZE * UI_SCALE)))
    return (
        load_ui_font(font_size),
        load_ui_font(big_size, bold=True),
    )


def configure_layout(window_w, window_h):
    global UI_SCALE, MARGIN, PANEL_W, ARENA_PX, PANEL_X, WIN_W, WIN_H, SCALE
    WIN_W = max(1, int(window_w))
    WIN_H = max(1, int(window_h))
    default_w = BASE_ARENA_PX + int(round(BASE_PANEL_W * BASE_UI_SCALE)) + 3 * int(round(BASE_MARGIN * BASE_UI_SCALE))
    default_h = BASE_ARENA_PX + 2 * int(round(BASE_MARGIN * BASE_UI_SCALE))
    size_factor = min(WIN_W / max(1, default_w), WIN_H / max(1, default_h))
    UI_SCALE = clamp(BASE_UI_SCALE * size_factor, MIN_UI_SCALE, MAX_UI_SCALE)
    MARGIN = ui(BASE_MARGIN)
    PANEL_W = ui(BASE_PANEL_W)
    available_w = max(MIN_ARENA_PX, WIN_W - PANEL_W - 3 * MARGIN)
    available_h = max(MIN_ARENA_PX, WIN_H - 2 * MARGIN)
    ARENA_PX = min(available_w, available_h)
    PANEL_X = MARGIN + ARENA_PX + MARGIN
    SCALE = ARENA_PX / ARENA_M


class Button:
    def __init__(self, rect, label, cb, is_on=None):
        self.rect = pygame.Rect(rect)
        self.label = label
        self.cb = cb
        self.is_on = is_on

    def draw(self, surf, font):
        on = self.is_on() if self.is_on else False
        m = self.rect.collidepoint(pygame.mouse.get_pos())
        col = BTN_ON if on else (BTN_HL if m else BTN)
        pygame.draw.rect(surf, col, self.rect, border_radius=ui(5))
        t = font.render(self.label, True, TXT)
        surf.blit(t, t.get_rect(center=self.rect.center))

    def click(self, pos):
        if self.rect.collidepoint(pos):
            self.cb()
            return True
        return False


class Slider:
    def __init__(self, label, get, set_, lo, hi, is_int=False, fmt="{:.2f}", tip=""):
        self.label = label
        self.get = get
        self.set = set_
        self.lo = lo
        self.hi = hi
        self.is_int = is_int
        self.fmt = fmt
        self.tip = tip
        self.label_rect = pygame.Rect(0, 0, 0, 0)
        self.rect = pygame.Rect(0, 0, 0, 0)
        self.drag = False

    def _to_px(self):
        frac = (self.get() - self.lo) / (self.hi - self.lo)
        return self.rect.x + int(clamp(frac, 0, 1) * self.rect.w)

    def _from_px(self, mx):
        frac = clamp((mx - self.rect.x) / max(1, self.rect.w), 0, 1)
        v = self.lo + frac * (self.hi - self.lo)
        if self.is_int:
            v = round(v)
        self.set(v)

    def draw(self, surf, font, x, y, w):
        track_h = max(4, ui(6))
        self.rect = pygame.Rect(x, y + ui(18), w, track_h)
        lab = font.render(self.label, True, TXT_DIM)
        self.label_rect = lab.get_rect(topleft=(x, y)).inflate(ui(8), ui(6))
        surf.blit(lab, (x, y))
        val = font.render(self.fmt.format(self.get()), True, TXT)
        surf.blit(val, (x + w - val.get_width(), y))
        pygame.draw.rect(surf, (60, 64, 78), self.rect, border_radius=max(2, ui(3)))
        hx = self._to_px()
        pygame.draw.circle(surf, (160, 200, 230), (hx, self.rect.centery), ui(7))

    def hover_tip(self, pos):
        if self.tip and self.label_rect.collidepoint(pos):
            return self.tip
        return None

    def handle(self, ev):
        if ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
            hit = self.rect.inflate(ui(8), ui(16)).collidepoint(ev.pos)
            if hit:
                self.drag = True
                self._from_px(ev.pos[0])
                return True
        elif ev.type == pygame.MOUSEBUTTONUP:
            self.drag = False
        elif ev.type == pygame.MOUSEMOTION and self.drag:
            self._from_px(ev.pos[0])
            return True
        return False


class Simulator:
    SIM_DT = 0.005

    def __init__(self):
        self.bridge = CodeBridge()
        self.p3 = self.bridge.p3
        self.arduino = ArduinoSim(self.bridge.ino)
        self.world = World()
        self.world.set_robot_radius(self.p3.ROBOT_RADIUS)

        self.running = False
        self.paused = False
        self.start_pose = (self.world.robot.x, self.world.robot.y, self.world.robot.theta)

        self.ctrl = self.p3.Controller()
        self.brain_acc = 0.0
        self.acc = 0.0
        self.sim_time = 0.0
        self.target_color = self.p3.TARGET_SEQUENCE[0]

        self.sim_speed = 1.0
        self.n_rays = 360
        self.lidar_noise = 0.0
        self.robot_height = 0.22
        self.camera_lift = 0.22
        self.obstacle_w = 0.30
        self.obstacle_h = 0.15
        self.obstacle_z_h = DEFAULT_OBSTACLE_Z_H
        self.patch_size = 0.35
        self._sync_project3_geometry()

        self.view_zoom = 1.0
        self.view_cx = ARENA_M * 0.5
        self.view_cy = ARENA_M * 0.5
        self.view_panning = False
        self.view_pan_last = None

        self.tele = dict(v=0.0, w=0.0, gb=0.0, clr=0.0, blocked=False,
                         see=False, npts=0, ar=0.0, bstate="SEEK")
        self.pts_world = np.empty((0, 2))
        self.traj_world = np.empty((0, 2))
        self.bear_pt = None
        self.run_history = []
        self.run_events = []
        self.history_start_wall_time = None
        self.history_sample_index = 0
        self.last_brain_snapshot = {
            "t_s": 0.0,
            "target_color": self.target_color,
            "state": "SEEK",
            "camera": {
                "visible": False,
                "bearing_rad": 0.0,
                "area_ratio": 0.0,
                "cy_norm": 0.0,
                "visible_candidate_count": 0,
                "target_center_robot_frame_m": None,
                "all_color_detections": {},
            },
            "lidar": {
                "raw_return_count": 0,
                "project3_filtered_point_count": 0,
                "nearest_project3_point_m": None,
            },
        }

        self.tool = "Obstacle"
        self.robot_placing = False
        self.obstacle_placing = False
        self.obstacle_anchor = None
        self.obstacle_preview = None
        self.param_scroll = 0
        self.param_scroll_drag = False
        self.param_scroll_drag_offset = 0
        self.toast = ("", 0.0)

        sim = self

        class _Link:
            def write(self, data):
                return sim.arduino.write(data)
        self.link = _Link()

        self._build_ui()

    def _sync_project3_geometry(self):
        """project3.py의 로봇/카메라 치수 상수를 시뮬레이터 기본값으로 반영."""
        self.robot_body_length = getattr(self.p3, "ROBOT_BODY_LENGTH_M", DEFAULT_ROBOT_BODY_LENGTH_M)
        self.robot_body_width = getattr(self.p3, "ROBOT_BODY_WIDTH_M", DEFAULT_ROBOT_BODY_WIDTH_M)
        self.camera_forward_offset = getattr(self.p3, "CAMERA_FORWARD_OFFSET_M", -0.05)
        self.camera_left_offset = getattr(self.p3, "CAMERA_LEFT_OFFSET_M", 0.0)
        self.camera_rear_blind = getattr(self.p3, "CAMERA_REAR_BLIND_M", 0.40)
        self.camera_width = getattr(self.p3, "CAMERA_RECT_WIDTH_M", 0.50)
        self.camera_depth = getattr(self.p3, "CAMERA_RECT_DEPTH_M", 0.89)
        self.world.set_robot_body_size(self.robot_body_length, self.robot_body_width)

    def resize(self, window_w, window_h):
        configure_layout(window_w, window_h)
        self._build_ui()
        self._clamp_param_scroll()

    def _build_ui(self):
        self.tool_buttons = []
        tool_rows = [
            [("Erase", "Erase"), ("Obstacle", "Obstacle"), ("Robot", "Robot")],
            [("Red", "RED"), ("Yellow", "YELLOW"), ("Blue", "BLUE")],
        ]
        gap = ui(10)
        btn_w = max(ui(96), (PANEL_W - 2 * gap) // 3)
        btn_h = ui(28)
        row_gap = ui(34)
        row_w = btn_w * 3 + gap * 2
        row_x = PANEL_X + max(0, (PANEL_W - row_w) // 2)
        for row_i, row in enumerate(tool_rows):
            y = MARGIN + row_i * row_gap
            for col_i, (label, value) in enumerate(row):
                x = row_x + col_i * (btn_w + gap)
                self.tool_buttons.append(
                    Button((x, y, btn_w, btn_h), label,
                           (lambda tt=value: self._set_tool(tt)),
                           is_on=(lambda tt=value: self.tool == tt)))

        self.action_buttons = []
        col_w = btn_w + gap
        acts = [("Start", self.start), ("Pause", self.toggle_pause),
                ("Reset", self.reset), ("Clear", self.clear),
                ("Save", self.save_layout), ("Load", self.load_layout)]
        for i, (lab, cb) in enumerate(acts):
            x = row_x + (i % 3) * col_w
            y = MARGIN + ui(78) + (i // 3) * row_gap
            on = None
            if lab == "Start":
                on = lambda: self.running and not self.paused
            elif lab == "Pause":
                on = lambda: self.paused
            self.action_buttons.append(Button((x, y, btn_w, btn_h), lab, cb, is_on=on))

        self.sign_button = Button(
            (PANEL_X, 0, ui(200), ui(28)), "",
            self._flip_bearing_sign,
            is_on=lambda: False)

        self.sliders = self._make_sliders()

    def _make_sliders(self):
        p3 = self.p3

        def p3set(attr):
            def f(v):
                setattr(self.p3, attr, v)
                if attr == "ROBOT_RADIUS":
                    self.world.set_robot_radius(v)
                recompute_derived(self.p3)
                if attr.startswith("EXPLORE_") and hasattr(self.ctrl, "_reset_explore"):
                    self.ctrl._reset_explore()
            return f

        def p3get(attr):
            return lambda: getattr(self.p3, attr)

        def ardget(attr):
            return lambda: getattr(self.arduino, attr)

        def ardset(attr):
            return lambda v: setattr(self.arduino, attr, v)

        def simget(attr):
            return lambda: getattr(self, attr)

        def simset(attr):
            return lambda v: setattr(self, attr, v)

        def set_robot_body_length(v):
            self.robot_body_length = v
            self.world.set_robot_body_size(self.robot_body_length, self.robot_body_width)
            if hasattr(self.p3, "ROBOT_BODY_LENGTH_M"):
                self.p3.ROBOT_BODY_LENGTH_M = v

        def set_robot_body_width(v):
            self.robot_body_width = v
            self.world.set_robot_body_size(self.robot_body_length, self.robot_body_width)
            if hasattr(self.p3, "ROBOT_BODY_WIDTH_M"):
                self.p3.ROBOT_BODY_WIDTH_M = v

        def p3simget(p3_attr, sim_attr):
            return lambda: getattr(self.p3, p3_attr, getattr(self, sim_attr))

        def p3simset(p3_attr, sim_attr):
            def f(v):
                setattr(self.p3, p3_attr, v)
                setattr(self, sim_attr, v)
                recompute_derived(self.p3)
            return f

        def S(label, get, set_, lo, hi, is_int=False, fmt="{:.2f}"):
            return Slider(
                label, get, set_, lo, hi,
                is_int=is_int, fmt=fmt, tip=PARAM_TIPS.get(label, "")
            )
        return [
            ("— DWA / robot (project3.py) —", None),
            ("CRUISE_V", S("CRUISE_V", p3get("CRUISE_V"), p3set("CRUISE_V"), 0.05, 2.00)),
            ("MAX_W", S("MAX_W", p3get("MAX_W"), p3set("MAX_W"), 0.3, 2.0)),
            ("W_RATE", S("W_RATE", p3get("W_RATE"), p3set("W_RATE"), 0.05, 1.0)),
            ("W_CLEAR", S("W_CLEAR", p3get("W_CLEAR"), p3set("W_CLEAR"), 0.0, 3.0)),
            ("W_GOAL", S("W_GOAL", p3get("W_GOAL"), p3set("W_GOAL"), 0.0, 3.0)),
            ("W_SPEED", S("W_SPEED", p3get("W_SPEED"), p3set("W_SPEED"), 0.0, 1.0)),
            ("W_TURN", S("W_TURN", p3get("W_TURN"), p3set("W_TURN"), 0.0, 1.0)),
            ("W_SMOOTH", S("W_SMOOTH", p3get("W_SMOOTH"), p3set("W_SMOOTH"), 0.0, 1.0)),
            ("HORIZON_T", S("HORIZON_T", p3get("HORIZON_T"), p3set("HORIZON_T"), 0.4, 2.5)),
            ("COLLISION_DIST", S("COLLISION_DIST", p3get("COLLISION_DIST"), p3set("COLLISION_DIST"), 0.10, 0.40)),
            ("SLOW_DIST", S("SLOW_DIST", p3get("SLOW_DIST"), p3set("SLOW_DIST"), 0.15, 0.80)),
            ("ROBOT_RADIUS", S("ROBOT_RADIUS", p3get("ROBOT_RADIUS"), p3set("ROBOT_RADIUS"), 0.08, 0.20)),
            ("KEEPIN_SIZE_M", S("KEEPIN_SIZE_M", p3get("KEEPIN_SIZE_M"), p3set("KEEPIN_SIZE_M"), 2.00, 3.00)),
            ("MAX_RANGE_M", S("MAX_RANGE_M", p3get("MAX_RANGE_M"), p3set("MAX_RANGE_M"), 0.5, 4.0)),
            ("V_MIN_RATIO", S("V_MIN_RATIO", p3get("V_MIN_RATIO"), p3set("V_MIN_RATIO"), 0.1, 1.0)),
            ("V_SET_MIN", S("V_SET_MIN", p3get("V_SET_MIN"), p3set("V_SET_MIN"), 0.00, 0.10, fmt="{:.3f}")),
            ("CLEAR_CAP", S("CLEAR_CAP", p3get("CLEAR_CAP"), p3set("CLEAR_CAP"), 0.2, 1.5)),
            ("GAP_BEARING_MAX", S("GAP_BEARING_MAX", p3get("GAP_BEARING_MAX"), p3set("GAP_BEARING_MAX"), 0.30, 1.80)),
            ("GAP_BEARING_STEP", S("GAP_BEARING_STEP", p3get("GAP_BEARING_STEP"), p3set("GAP_BEARING_STEP"), 0.05, 0.40, fmt="{:.3f}")),
            ("GAP_WINDOW_RAD", S("GAP_WINDOW_RAD", p3get("GAP_WINDOW_RAD"), p3set("GAP_WINDOW_RAD"), 0.05, 0.60, fmt="{:.3f}")),
            ("GAP_MIN_CLEAR_MARGIN", S("GAP_MIN_CLEAR_MARGIN", p3get("GAP_MIN_CLEAR_MARGIN"), p3set("GAP_MIN_CLEAR_MARGIN"), 0.00, 0.12, fmt="{:.3f}")),
            ("GAP_PASS_LOOKAHEAD_M", S("GAP_PASS_LOOKAHEAD_M", p3get("GAP_PASS_LOOKAHEAD_M"), p3set("GAP_PASS_LOOKAHEAD_M"), 0.20, 1.50)),
            ("GAP_PASS_SIDE_MARGIN_M", S("GAP_PASS_SIDE_MARGIN_M", p3get("GAP_PASS_SIDE_MARGIN_M"), p3set("GAP_PASS_SIDE_MARGIN_M"), 0.00, 0.15, fmt="{:.3f}")),
            ("GAP_W_PASS", S("GAP_W_PASS", p3get("GAP_W_PASS"), p3set("GAP_W_PASS"), 0.00, 3.00)),
            ("AVOID_TRIGGER_MARGIN", S("AVOID_TRIGGER_MARGIN", p3get("AVOID_TRIGGER_MARGIN"), p3set("AVOID_TRIGGER_MARGIN"), 0.00, 0.12, fmt="{:.3f}")),
            ("AVOID_RELEASE_MARGIN", S("AVOID_RELEASE_MARGIN", p3get("AVOID_RELEASE_MARGIN"), p3set("AVOID_RELEASE_MARGIN"), 0.00, 0.20, fmt="{:.3f}")),
            ("AVOID_LOST_MEMORY_S", S("AVOID_LOST_MEMORY_S", p3get("AVOID_LOST_MEMORY_S"), p3set("AVOID_LOST_MEMORY_S"), 0.00, 4.00)),
            ("ESCAPE_ROTATE_MARGIN", S("ESCAPE_ROTATE_MARGIN", p3get("ESCAPE_ROTATE_MARGIN"), p3set("ESCAPE_ROTATE_MARGIN"), 0.00, 0.08, fmt="{:.3f}")),
            ("ESCAPE_CREEP_MAX_V", S("ESCAPE_CREEP_MAX_V", p3get("ESCAPE_CREEP_MAX_V"), p3set("ESCAPE_CREEP_MAX_V"), 0.00, 0.10, fmt="{:.3f}")),
            ("ESCAPE_REVERSE_V", S("ESCAPE_REVERSE_V", p3get("ESCAPE_REVERSE_V"), p3set("ESCAPE_REVERSE_V"), 0.00, 0.10, fmt="{:.3f}")),
            ("ESCAPE_REVERSE_MIN_GAIN", S("ESCAPE_REVERSE_MIN_GAIN", p3get("ESCAPE_REVERSE_MIN_GAIN"), p3set("ESCAPE_REVERSE_MIN_GAIN"), 0.00, 0.02, fmt="{:.3f}")),
            ("HFOV_DEG", S("HFOV_DEG", p3get("HFOV_DEG"), p3set("HFOV_DEG"), 30.0, 120.0)),
            ("MIN_AREA", S("MIN_AREA", p3get("MIN_AREA"), p3set("MIN_AREA"), 0, 500, is_int=True, fmt="{:.0f}")),
            ("APPROACH_CY", S("APPROACH_CY", p3get("APPROACH_CY"), p3set("APPROACH_CY"), 0.40, 0.95)),
            ("APPROACH_ALIGN_MAX", S("APPROACH_ALIGN_MAX", p3get("APPROACH_ALIGN_MAX"), p3set("APPROACH_ALIGN_MAX"), 0.03, 0.35, fmt="{:.3f}")),
            ("APPROACH_STABLE_S", S("APPROACH_STABLE_S", p3get("APPROACH_STABLE_S"), p3set("APPROACH_STABLE_S"), 0.00, 1.00)),
            ("CLOSE_LOST_HOLD_CY", S("CLOSE_LOST_HOLD_CY", p3get("CLOSE_LOST_HOLD_CY"), p3set("CLOSE_LOST_HOLD_CY"), 0.70, 1.00)),
            ("BLIND_CREEP_DIST_M", S("BLIND_CREEP_DIST_M", p3get("BLIND_CREEP_DIST_M"), p3set("BLIND_CREEP_DIST_M"), 0.02, 0.25, fmt="{:.3f}")),
            ("CREEP_STEER_GAIN", S("CREEP_STEER_GAIN", p3get("CREEP_STEER_GAIN"), p3set("CREEP_STEER_GAIN"), 0.00, 1.20)),
            ("CREEP_MAX_W", S("CREEP_MAX_W", p3get("CREEP_MAX_W"), p3set("CREEP_MAX_W"), 0.00, 0.50)),
            ("DWELL_S", S("DWELL_S", p3get("DWELL_S"), p3set("DWELL_S"), 1.00, 2.00)),
            ("CENTER_TOL_M", S("CENTER_TOL_M", p3get("CENTER_TOL_M"), p3set("CENTER_TOL_M"), 0.005, 0.080, fmt="{:.3f}")),
            ("CENTER_MAX_V", S("CENTER_MAX_V", p3get("CENTER_MAX_V"), p3set("CENTER_MAX_V"), 0.03, 0.20, fmt="{:.3f}")),
            ("CENTER_SLOW_RADIUS_M", S("CENTER_SLOW_RADIUS_M", p3get("CENTER_SLOW_RADIUS_M"), p3set("CENTER_SLOW_RADIUS_M"), 0.08, 0.80)),
            ("CENTER_GOAL_ALPHA", S("CENTER_GOAL_ALPHA", p3get("CENTER_GOAL_ALPHA"), p3set("CENTER_GOAL_ALPHA"), 0.05, 1.00)),
            ("CENTER_LOST_MEMORY_S", S("CENTER_LOST_MEMORY_S", p3get("CENTER_LOST_MEMORY_S"), p3set("CENTER_LOST_MEMORY_S"), 0.00, 5.00)),
            ("CENTER_MAX_BEARING", S("CENTER_MAX_BEARING", p3get("CENTER_MAX_BEARING"), p3set("CENTER_MAX_BEARING"), 0.20, 1.60)),
            ("SEARCH_W", S("SEARCH_W", p3get("SEARCH_W"), p3set("SEARCH_W"), 0.20, 1.50)),
            ("SEARCH_SWEEP_ANGLE_DEG", S("SEARCH_SWEEP_ANGLE_DEG", p3get("SEARCH_SWEEP_ANGLE_DEG"), p3set("SEARCH_SWEEP_ANGLE_DEG"), 10.0, 90.0)),
            ("SEARCH_SETTLE_RAD", S("SEARCH_SETTLE_RAD", p3get("SEARCH_SETTLE_RAD"), p3set("SEARCH_SETTLE_RAD"), 0.02, 0.25, fmt="{:.3f}")),
            ("SEARCH_SCAN_EDGE_HITS", S("SEARCH_SCAN_EDGE_HITS", p3get("SEARCH_SCAN_EDGE_HITS"), p3set("SEARCH_SCAN_EDGE_HITS"), 1, 6, is_int=True, fmt="{:.0f}")),
            ("SEARCH_DRIVE_V", S("SEARCH_DRIVE_V", p3get("SEARCH_DRIVE_V"), p3set("SEARCH_DRIVE_V"), 0.03, 0.25)),
            ("SEARCH_DRIVE_S", S("SEARCH_DRIVE_S", p3get("SEARCH_DRIVE_S"), p3set("SEARCH_DRIVE_S"), 0.20, 3.00)),
            ("EXPLORE_SPACING_M", S("EXPLORE_SPACING_M", p3get("EXPLORE_SPACING_M"), p3set("EXPLORE_SPACING_M"), 0.20, 0.60)),
            ("EXPLORE_REACH_DIST_M", S("EXPLORE_REACH_DIST_M", p3get("EXPLORE_REACH_DIST_M"), p3set("EXPLORE_REACH_DIST_M"), 0.05, 0.25)),
            ("EXPLORE_SCAN_EDGE_HITS", S("EXPLORE_SCAN_EDGE_HITS", p3get("EXPLORE_SCAN_EDGE_HITS"), p3set("EXPLORE_SCAN_EDGE_HITS"), 0, 6, is_int=True, fmt="{:.0f}")),
            ("EXPLORE_MAX_V", S("EXPLORE_MAX_V", p3get("EXPLORE_MAX_V"), p3set("EXPLORE_MAX_V"), 0.03, 0.25)),
            ("EXPLORE_SLOW_DIST_M", S("EXPLORE_SLOW_DIST_M", p3get("EXPLORE_SLOW_DIST_M"), p3set("EXPLORE_SLOW_DIST_M"), 0.10, 0.60)),
            ("EXPLORE_BLOCKED_S", S("EXPLORE_BLOCKED_S", p3get("EXPLORE_BLOCKED_S"), p3set("EXPLORE_BLOCKED_S"), 0.20, 3.00)),
            ("EXPLORE_STUCK_S", S("EXPLORE_STUCK_S", p3get("EXPLORE_STUCK_S"), p3set("EXPLORE_STUCK_S"), 0.50, 6.00)),
            ("WRONG_COLOR_AVOID_RADIUS_M", S("WRONG_COLOR_AVOID_RADIUS_M", p3get("WRONG_COLOR_AVOID_RADIUS_M"), p3set("WRONG_COLOR_AVOID_RADIUS_M"), 0.10, 1.20)),
            ("WRONG_COLOR_SCORE_PENALTY", S("WRONG_COLOR_SCORE_PENALTY", p3get("WRONG_COLOR_SCORE_PENALTY"), p3set("WRONG_COLOR_SCORE_PENALTY"), 0.00, 4.00)),
            ("TARGET_MEMORY_REACHED_DIST_M", S("TARGET_MEMORY_REACHED_DIST_M", p3get("TARGET_MEMORY_REACHED_DIST_M"), p3set("TARGET_MEMORY_REACHED_DIST_M"), 0.10, 0.80)),
            ("— Arduino (project3.ino) —", None),
            ("wheel tau", S("wheel tau", ardget("tau"), ardset("tau"), 0.02, 0.40)),
            ("wheel accel", S("wheel accel", ardget("wheel_accel_max"), ardset("wheel_accel_max"), 5.0, 100.0)),
            ("— Simulator —", None),
            ("sim speed", S("sim speed", simget("sim_speed"), simset("sim_speed"), 0.25, 3.0)),
            ("lidar noise", S("lidar noise", simget("lidar_noise"), simset("lidar_noise"), 0.0, 0.03, fmt="{:.3f}")),
            ("n_rays", S("n_rays", simget("n_rays"), simset("n_rays"), 90, 720, is_int=True, fmt="{:.0f}")),
            ("robot height", S("robot height", simget("robot_height"), simset("robot_height"), 0.08, 0.50)),
            ("robot length", S("robot length", simget("robot_body_length"), set_robot_body_length, 0.08, 0.40)),
            ("robot width", S("robot width", simget("robot_body_width"), set_robot_body_width, 0.08, 0.40)),
            ("camera lift", S("camera lift", simget("camera_lift"), simset("camera_lift"), 0.00, 0.60)),
            ("cam forward ofs", S("cam forward ofs", p3simget("CAMERA_FORWARD_OFFSET_M", "camera_forward_offset"), p3simset("CAMERA_FORWARD_OFFSET_M", "camera_forward_offset"), -0.30, 0.30)),
            ("cam left ofs", S("cam left ofs", p3simget("CAMERA_LEFT_OFFSET_M", "camera_left_offset"), p3simset("CAMERA_LEFT_OFFSET_M", "camera_left_offset"), -0.30, 0.30)),
            ("cam rear blind", S("cam rear blind", p3simget("CAMERA_REAR_BLIND_M", "camera_rear_blind"), p3simset("CAMERA_REAR_BLIND_M", "camera_rear_blind"), 0.00, 0.80)),
            ("cam width", S("cam width", p3simget("CAMERA_RECT_WIDTH_M", "camera_width"), p3simset("CAMERA_RECT_WIDTH_M", "camera_width"), 0.10, 1.20)),
            ("cam depth", S("cam depth", p3simget("CAMERA_RECT_DEPTH_M", "camera_depth"), p3simset("CAMERA_RECT_DEPTH_M", "camera_depth"), 0.10, 1.50)),
            ("obstacle w", S("obstacle w", simget("obstacle_w"), simset("obstacle_w"), 0.05, 0.80)),
            ("obstacle depth", S("obstacle depth", simget("obstacle_h"), simset("obstacle_h"), 0.05, 0.50)),
            ("obstacle z", S("obstacle z", simget("obstacle_z_h"), simset("obstacle_z_h"), 0.05, 0.50)),
            ("patch size", S("patch size", simget("patch_size"), simset("patch_size"), 0.10, 0.50)),
        ]

    def _set_tool(self, t):
        self.tool = t
        self.robot_placing = False
        self.obstacle_placing = False
        self.obstacle_anchor = None
        self.obstacle_preview = None

    def _flip_bearing_sign(self):
        self.p3.BEARING_SIGN = -self.p3.BEARING_SIGN

    def _say(self, msg):
        self.toast = (msg, 2.5)

    def start(self):
        if not self.running:
            self.start_pose = (self.world.robot.x, self.world.robot.y, self.world.robot.theta)
            self._begin_run_history("start")
        elif self.paused:
            self._history_event("resume")
        self.running = True
        self.paused = False
        self._say("RUN")

    def toggle_pause(self):
        if self.running:
            self.paused = not self.paused
            self._history_event("pause" if self.paused else "resume")

    def reset(self):
        self.running = False
        self.paused = False
        x, y, th = self.start_pose
        self.world.reset_run(x, y, th)
        self.arduino.V_cmd = self.arduino.W_cmd = 0.0
        self.arduino.wL = self.arduino.wR = 0.0
        self.ctrl = self.p3.Controller()
        self.brain_acc = self.acc = 0.0
        self.sim_time = 0.0
        self.target_color = self.p3.TARGET_SEQUENCE[0]
        self._begin_run_history("reset")
        self._say("RESET")

    def clear(self):
        self.world.obstacles.clear()
        self.world.patches.clear()
        self._history_event("clear_layout")
        self._say("CLEARED")

    def save_layout(self):
        data = self._make_save_data()
        os.makedirs(DATA_DIR, exist_ok=True)
        with open(LAYOUT_PATH, "w") as f:
            json.dump(data, f, indent=2, ensure_ascii=False, default=json_default)
        self._say("SAVED data/layout.json")

    def _make_save_data(self):
        data = {
            "schema": "drive_sim_layout_v6",
            "saved_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "note": (
                "robot/obstacles/patches are the compact loadable layout. "
                "report contains the full simulator context, full run history, and failure diagnostics "
                "for Codex/Claude-style code analysis."
            ),
            "robot": [self.world.robot.x, self.world.robot.y, self.world.robot.theta],
            "obstacles": [[o.x, o.y, o.w, o.h, o.theta, o.z_h] for o in self.world.obstacles],
            "patches": [[p.x, p.y, p.color, p.size] for p in self.world.patches],
            "report": self._make_layout_report(),
        }
        return data

    def load_layout(self):
        path = LAYOUT_PATH if os.path.exists(LAYOUT_PATH) else LEGACY_LAYOUT_PATH
        if not os.path.exists(path):
            self._say("no layout.json")
            return
        with open(path) as f:
            d = json.load(f)
        rb = d["robot"]
        self.world.robot.x, self.world.robot.y, self.world.robot.theta = rb
        self.world.obstacles = [self._load_obstacle(o) for o in d.get("obstacles", [])]
        self.world.patches = [self._load_patch(p) for p in d.get("patches", [])]
        self.start_pose = tuple(rb)
        self._begin_run_history("load_layout")
        label = "data/layout.json" if path == LAYOUT_PATH else "layout.json"
        self._say("LOADED " + label)

    def _load_obstacle(self, data):
        if isinstance(data, dict):
            center = data.get("center_m", data.get("center", data))
            if isinstance(center, dict) and "world_m" in center:
                center = center["world_m"]
            size = data.get("footprint_m", data.get("size_m", data))
            x = center.get("x_m", center.get("x", 0.0))
            y = center.get("y_m", center.get("y", 0.0))
            w = size.get("width_m", size.get("w_m", data.get("w", self.obstacle_w)))
            h = size.get("depth_m", size.get("height_m", size.get("h_m", data.get("h", self.obstacle_h))))
            theta = data.get("theta_rad", math.radians(data.get("theta_deg", 0.0)))
            z_h = data.get(
                "vertical_height_m",
                data.get("z_h_m", data.get("z_h", self.obstacle_z_h)),
            )
            return Obstacle(x, y, w, h, theta, z_h)
        if len(data) >= 6:
            return Obstacle(data[0], data[1], data[2], data[3], data[4], data[5])
        if len(data) >= 5:
            return Obstacle(data[0], data[1], data[2], data[3], data[4], self.obstacle_z_h)
        if len(data) >= 4:
            return Obstacle(data[0], data[1], data[2], data[3], 0.0, self.obstacle_z_h)
        if len(data) == 3:
            return Obstacle(data[0], data[1], data[2] * 2.0, data[2] * 2.0, 0.0, self.obstacle_z_h)
        return Obstacle(*data)

    def _load_patch(self, data):
        if isinstance(data, dict):
            center = data.get("center_m", data)
            x = center.get("x_m", center.get("x", 0.0))
            y = center.get("y_m", center.get("y", 0.0))
            color = data.get("color", "RED")
            size = data.get("size_m", data.get("size", self.patch_size))
            return Patch(x, y, color, size)
        return Patch(data[0], data[1], data[2], data[3])

    def _compact_layout_snapshot(self):
        return {
            "robot": [self.world.robot.x, self.world.robot.y, self.world.robot.theta],
            "obstacles": [
                [o.x, o.y, o.w, o.h, o.theta, o.z_h]
                for o in self.world.obstacles
            ],
            "patches": [
                [p.x, p.y, p.color, p.size]
                for p in self.world.patches
            ],
        }

    def _history_event(self, event, details=None):
        self.run_events.append({
            "t_s": self.sim_time,
            "event": event,
            "details": details or {},
        })

    def _begin_run_history(self, reason):
        self.run_history = []
        self.run_events = []
        self.history_sample_index = 0
        self.history_start_wall_time = datetime.now().astimezone().isoformat(timespec="seconds")
        self.last_brain_snapshot = {
            "t_s": self.sim_time,
            "target_color": self.target_color,
            "state": self.tele.get("bstate", "SEEK"),
            "camera": {
                "visible": bool(self.tele.get("see", False)),
                "bearing_rad": self.tele.get("gb", 0.0),
                "area_ratio": self.tele.get("ar", 0.0),
                "cy_norm": 0.0,
                "visible_candidate_count": 0,
                "target_center_robot_frame_m": None,
                "all_color_detections": {},
            },
            "lidar": {
                "raw_return_count": 0,
                "project3_filtered_point_count": self.tele.get("npts", 0),
                "nearest_project3_point_m": None,
            },
        }
        self._history_event(reason, {
            "layout": self._compact_layout_snapshot(),
            "start_pose": list(self.start_pose),
        })
        self._append_history_sample(reason)

    def _append_history_sample(self, reason):
        rb = self.world.robot
        ctrl = self.ctrl
        sample = {
            "i": self.history_sample_index,
            "t_s": self.sim_time,
            "reason": reason,
            "robot": {
                "world_m": {
                    "x_m": rb.x,
                    "y_m": rb.y,
                    "theta_rad": rb.theta,
                    "theta_deg": math.degrees(rb.theta),
                },
                "centered_m": {
                    "x_m": rb.x - ARENA_M * 0.5,
                    "y_m": rb.y - ARENA_M * 0.5,
                    "theta_rad": rb.theta,
                    "theta_deg": math.degrees(rb.theta),
                },
                "velocity": {"v_mps": rb.v, "w_radps": rb.w},
            },
            "command": {
                "v_mps": self.tele.get("v", 0.0),
                "w_radps": self.tele.get("w", 0.0),
                "state": self.tele.get("bstate", None),
                "goal_bearing_rad": self.tele.get("gb", 0.0),
                "clearance_m": self.tele.get("clr", 0.0),
                "blocked": bool(self.tele.get("blocked", False)),
                "avoid_active": bool(self.tele.get("avoid_active", False)),
                "avoid_goal_rad": self.tele.get("avoid_goal", 0.0),
                "direct_clearance_m": self.tele.get("avoid_direct_clr", None),
                "target_sector_clearance_m": self.tele.get("avoid_target_sector_clear", None),
                "gap_sector_clearance_m": self.tele.get("avoid_sector_clear", None),
                "gap_pass_clearance_m": self.tele.get("avoid_pass_clear", None),
            },
            "controller": {
                "seq_idx": getattr(ctrl, "seq_idx", None),
                "target_color": getattr(ctrl, "target_color", None),
                "phase": getattr(ctrl, "phase", None),
                "done": getattr(ctrl, "done", False),
                "last_w": getattr(ctrl, "last_w", None),
                "goal": getattr(ctrl, "goal", None),
                "clr": getattr(ctrl, "clr", None),
                "search_dir": getattr(ctrl, "search_dir", None),
                "search_switch_t": getattr(ctrl, "search_switch_t", None),
                "search_anchor_theta": getattr(ctrl, "search_anchor_theta", None),
                "search_mode": getattr(ctrl, "search_mode", None),
                "search_edge_hits": getattr(ctrl, "search_edge_hits", None),
                "search_force_explore": getattr(ctrl, "search_force_explore", None),
                "search_drive_until": getattr(ctrl, "search_drive_until", None),
                "explore_target_idx": getattr(ctrl, "explore_target_idx", None),
                "explore_target_world_m": (
                    {
                        "x_m": ctrl.explore_waypoints[ctrl.explore_target_idx][0],
                        "y_m": ctrl.explore_waypoints[ctrl.explore_target_idx][1],
                    }
                    if getattr(ctrl, "explore_target_idx", None) is not None
                    and 0 <= ctrl.explore_target_idx < len(getattr(ctrl, "explore_waypoints", []))
                    else None
                ),
                "explore_cycle": getattr(ctrl, "explore_cycle", None),
                "explore_visited_count": sum(1 for v in getattr(ctrl, "explore_visited", []) if v),
                "explore_deferred_count": sum(1 for v in getattr(ctrl, "explore_deferred", []) if v),
                "explore_waypoint_count": len(getattr(ctrl, "explore_waypoints", [])),
                "explore_priority_count": getattr(ctrl, "explore_priority_count", None),
                "color_memory": dict(getattr(ctrl, "color_memory", {})),
                "avoid_active": getattr(ctrl, "avoid_active", None),
                "avoid_goal": getattr(ctrl, "avoid_goal", None),
                "avoid_direct_clr": getattr(ctrl, "avoid_direct_clr", None),
                "avoid_target_sector_clear": getattr(ctrl, "avoid_target_sector_clear", None),
                "avoid_sector_clear": getattr(ctrl, "avoid_sector_clear", None),
                "avoid_pass_clear": getattr(ctrl, "avoid_pass_clear", None),
                "avoid_required_half_width": getattr(ctrl, "avoid_required_half_width", None),
                "avoid_lookahead": getattr(ctrl, "avoid_lookahead", None),
                "avoid_score": getattr(ctrl, "avoid_score", None),
                "last_seen_t": getattr(ctrl, "last_seen_t", None),
                "last_seen_bearing": getattr(ctrl, "last_seen_bearing", None),
                "last_seen_cy": getattr(ctrl, "last_seen_cy", None),
                "smooth_bearing": getattr(ctrl, "smooth_bearing", None),
                "close_peak_cy": getattr(ctrl, "close_peak_cy", None),
                "approach_align_since": getattr(ctrl, "approach_align_since", None),
                "approach_ready": getattr(ctrl, "approach_ready", None),
                "center_goal_world_m": (
                    {
                        "x_m": getattr(ctrl, "center_goal_x", None),
                        "y_m": getattr(ctrl, "center_goal_y", None),
                    }
                    if getattr(ctrl, "center_goal_x", None) is not None else None
                ),
                "last_center_error_robot_frame_m": (
                    {
                        "x_m": getattr(ctrl, "last_target_x", None),
                        "y_m": getattr(ctrl, "last_target_y", None),
                        "distance_m": getattr(ctrl, "last_center_dist", None),
                    }
                    if getattr(ctrl, "last_center_dist", None) is not None else None
                ),
            },
            "keepin": self._keepin_report(),
            "judge": {
                "required_target": self.world.current_target_color(),
                "cleared_by_order": dict(zip(ORDER, self.world.cleared)),
                "dwell_s": self.world.dwell,
                "collisions": self.world.collisions,
                "finished": self.world.finished,
            },
            "arduino": {
                "V_cmd": self.arduino.V_cmd,
                "W_cmd": self.arduino.W_cmd,
                "wL": self.arduino.wL,
                "wR": self.arduino.wR,
            },
            "sensors": copy.deepcopy(self.last_brain_snapshot),
        }
        self.run_history.append(sample)
        self.history_sample_index += 1

    def _run_history_report(self):
        return {
            "recording": "complete physics-step pose/command trace for this simulator session",
            "sample_period_s": self.SIM_DT,
            "history_start_wall_time": self.history_start_wall_time,
            "sample_count": len(self.run_history),
            "event_count": len(self.run_events),
            "events": self.run_events,
            "samples": self.run_history,
        }

    def _centered_point(self, x, y):
        return {
            "x_m": x - ARENA_M * 0.5,
            "y_m": y - ARENA_M * 0.5,
        }

    def _point_report(self, x, y):
        return {
            "world_m": {"x_m": x, "y_m": y},
            "centered_m": self._centered_point(x, y),
        }

    def _pose_report(self, x, y, theta):
        return {
            "world_m": {"x_m": x, "y_m": y, "theta_rad": theta, "theta_deg": math.degrees(theta)},
            "centered_m": {
                "x_m": x - ARENA_M * 0.5,
                "y_m": y - ARENA_M * 0.5,
                "theta_rad": theta,
                "theta_deg": math.degrees(theta),
            },
        }

    def _p3_pose_tuple(self):
        rb = self.world.robot
        return (rb.x - ARENA_M * 0.5, rb.y - ARENA_M * 0.5, rb.theta)

    def _keepin_bounds_world(self):
        size = getattr(self.p3, "KEEPIN_SIZE_M", 0.0)
        half = size * 0.5
        cx = ARENA_M * 0.5
        cy = ARENA_M * 0.5
        return {
            "x_min_m": cx - half,
            "x_max_m": cx + half,
            "y_min_m": cy - half,
            "y_max_m": cy + half,
        }

    def _keepin_report(self):
        size = getattr(self.p3, "KEEPIN_SIZE_M", None)
        margin = getattr(self.p3, "KEEPIN_MARGIN_M", None)
        enabled = bool(getattr(self.p3, "KEEPIN_ENABLED", False))
        bounds = self._keepin_bounds_world()
        centered_half = (size * 0.5) if size is not None else None
        effective_half = None
        if size is not None and margin is not None:
            effective_half = size * 0.5 - margin
        current_margin = None
        predicted_margin = None
        if hasattr(self.p3, "keepin_boundary_margin"):
            pose = self._p3_pose_tuple()
            current_margin = self.p3.keepin_boundary_margin(0.0, 0.0, pose)
            predicted_margin = self.p3.keepin_boundary_margin(
                max(self.tele.get("v", 0.0), 0.05), self.tele.get("w", 0.0), pose)
        return {
            "enabled": enabled,
            "size_m": size,
            "margin_m": margin,
            "world_bounds_m": bounds,
            "centered_bounds_m": {
                "x_min_m": -centered_half if centered_half is not None else None,
                "x_max_m": centered_half,
                "y_min_m": -centered_half if centered_half is not None else None,
                "y_max_m": centered_half,
            },
            "effective_center_bounds_m": {
                "x_min_m": -effective_half if effective_half is not None else None,
                "x_max_m": effective_half,
                "y_min_m": -effective_half if effective_half is not None else None,
                "y_max_m": effective_half,
            },
            "current_center_margin_m": current_margin,
            "predicted_command_margin_m": predicted_margin,
        }

    def _camera_rect_half_width(self):
        return sensors.camera_rect_half_width(self.p3, cam_width_m=self.camera_width)

    def _camera_forward_range(self):
        return sensors.camera_forward_range(
            cam_rear_blind_m=self.camera_rear_blind,
            cam_depth_m=self.camera_depth,
            robot_radius_m=self.p3.ROBOT_RADIUS,
            robot_body_length_m=self.robot_body_length,
            camera_forward_offset_m=self.camera_forward_offset,
        )

    def _camera_mount_height(self):
        return self.robot_height + self.camera_lift

    def _camera_world_pose(self):
        rb = self.world.robot
        return sensors.camera_world_pose(
            rb, self.camera_forward_offset, self.camera_left_offset)

    def _camera_rect_world_points(self):
        rb = self.world.robot
        half_w = self._camera_rect_half_width()
        near_x, far_x = self._camera_forward_range()
        far_x = max(near_x + 1e-6, far_x)
        cam_x, cam_y = self._camera_world_pose()
        cs, sn = math.cos(rb.theta), math.sin(rb.theta)
        pts = []
        for lx, ly in (
            (near_x, -half_w),
            (far_x, -half_w),
            (far_x, half_w),
            (near_x, half_w),
        ):
            pts.append((cam_x + lx * cs - ly * sn,
                        cam_y + lx * sn + ly * cs))
        return pts

    def _robot_body_world_points(self):
        self.world.set_robot_body_size(self.robot_body_length, self.robot_body_width)
        return self.world.robot.corners()

    def _segment_distance(self, a, b, c, d):
        def point_segment_distance(p, q0, q1):
            vx, vy = q1[0] - q0[0], q1[1] - q0[1]
            wx, wy = p[0] - q0[0], p[1] - q0[1]
            vv = vx * vx + vy * vy
            if vv <= 1e-12:
                return math.hypot(p[0] - q0[0], p[1] - q0[1])
            t = clamp((wx * vx + wy * vy) / vv, 0.0, 1.0)
            px, py = q0[0] + t * vx, q0[1] + t * vy
            return math.hypot(p[0] - px, p[1] - py)

        return min(
            point_segment_distance(a, c, d),
            point_segment_distance(b, c, d),
            point_segment_distance(c, a, b),
            point_segment_distance(d, a, b),
        )

    def _polygon_distance(self, a, b):
        if polygons_intersect(a, b):
            return 0.0
        best = float("inf")
        for i, a0 in enumerate(a):
            a1 = a[(i + 1) % len(a)]
            for j, b0 in enumerate(b):
                b1 = b[(j + 1) % len(b)]
                best = min(best, self._segment_distance(a0, a1, b0, b1))
        return best

    def _robot_obstacle_clearance(self, obstacle):
        return self._polygon_distance(self._robot_body_world_points(), obstacle.corners())

    def _patch_bounds(self, p):
        h = p.size * 0.5
        return {
            "x_min_m": p.x - h,
            "x_max_m": p.x + h,
            "y_min_m": p.y - h,
            "y_max_m": p.y + h,
        }

    def _obstacle_report(self, o, idx):
        rb = self.world.robot
        body_clearance = self._robot_obstacle_clearance(o)
        corners = [self._point_report(x, y) for x, y in o.corners()]
        return {
            "id": "obstacle_{:02d}".format(idx + 1),
            "type": "rotated_rectangle",
            "center": self._point_report(o.x, o.y),
            "footprint_m": {"width_m": o.w, "depth_m": o.h},
            "vertical_height_m": o.z_h,
            "theta_rad": o.theta,
            "theta_deg": math.degrees(o.theta),
            "corners": corners,
            "distance_to_robot_center_m": o.distance_to_point(rb.x, rb.y),
            "clearance_from_robot_body_m": body_clearance,
            "intersects_robot_body": body_clearance <= 1e-9,
        }

    def _patch_report(self, p, idx, left_wheel, right_wheel):
        order_idx = ORDER.index(p.color) if p.color in ORDER else None
        return {
            "id": "patch_{:02d}_{}".format(idx + 1, p.color),
            "color": p.color,
            "center": self._point_report(p.x, p.y),
            "size_m": p.size,
            "bounds_m": self._patch_bounds(p),
            "target_order_index": order_idx,
            "left_wheel_inside": p.contains(*left_wheel),
            "right_wheel_inside": p.contains(*right_wheel),
            "both_wheels_inside": p.contains(*left_wheel) and p.contains(*right_wheel),
        }

    def _nearest_obstacle_snapshot(self):
        rb = self.world.robot
        best = None
        for i, o in enumerate(self.world.obstacles):
            d_center = o.distance_to_point(rb.x, rb.y)
            clearance = self._robot_obstacle_clearance(o)
            item = {
                "id": "obstacle_{:02d}".format(i + 1),
                "distance_to_robot_center_m": d_center,
                "clearance_from_robot_body_m": clearance,
                "center": self._point_report(o.x, o.y),
                "theta_deg": math.degrees(o.theta),
                "footprint_m": {"width_m": o.w, "depth_m": o.h},
                "vertical_height_m": o.z_h,
            }
            if best is None or clearance < best["clearance_from_robot_body_m"]:
                best = item
        return best

    def _camera_blocker_report(self, patch, target_xy=None):
        cam_x, cam_y = self._camera_world_pose()
        target_x, target_y = target_xy if target_xy is not None else (patch.x, patch.y)
        blocked = sensors.camera_blocker(
            self.world, cam_x, cam_y, target_x, target_y, self._camera_mount_height())
        if blocked is None:
            return None
        obstacle, frac, line_z = blocked
        obstacle_idx = None
        for i, candidate in enumerate(self.world.obstacles):
            if candidate is obstacle:
                obstacle_idx = i
                break
        d = math.hypot(target_x - cam_x, target_y - cam_y)
        return {
            "obstacle_id": (
                "obstacle_{:02d}".format(obstacle_idx + 1)
                if obstacle_idx is not None else None
            ),
            "target_point": self._point_report(target_x, target_y),
            "fraction_along_camera_ray": frac,
            "distance_from_camera_m": frac * d,
            "camera_ray_height_at_blocker_m": line_z,
            "obstacle_vertical_height_m": obstacle.z_h,
        }

    def _target_geometry(self, color):
        rb = self.world.robot
        cs, sn = math.cos(rb.theta), math.sin(rb.theta)
        half_fov = self.p3.HALF_HFOV_RAD
        focal_px = (self.p3.CAM_W * 0.5) / max(1e-6, math.tan(half_fov))
        half_width = self._camera_rect_half_width()
        near_x, far_x = self._camera_forward_range()
        cam_x, cam_y = self._camera_world_pose()
        targets = []
        for i, p in enumerate(self.world.patches):
            if p.color != color:
                continue
            robot_rx, robot_ry = p.x - rb.x, p.y - rb.y
            robot_x = robot_rx * cs + robot_ry * sn
            robot_y = -robot_rx * sn + robot_ry * cs
            rx, ry = p.x - cam_x, p.y - cam_y
            x_c = rx * cs + ry * sn
            y_c = -rx * sn + ry * cs
            distance = math.hypot(x_c, y_c)
            bearing = math.atan2(y_c, x_c) if distance > 1e-9 else 0.0
            projection_distance = x_c if x_c > 0.0 else distance
            side_px = focal_px * p.size / max(1e-6, projection_distance)
            area_px = side_px * side_px
            center_blocker = self._camera_blocker_report(p)
            image_err = -clamp(y_c / half_width, -1.0, 1.0)
            simulated_bearing = self.p3.BEARING_SIGN * image_err * half_fov
            inside_width = abs(y_c) <= half_width
            inside_forward_range = near_x <= x_c <= far_x
            overlap = sensors.patch_camera_overlap(
                self.world, p, near_x, far_x, half_width,
                self.camera_forward_offset, self.camera_left_offset)
            overlap_visible = bool(overlap["visible"])
            overlap_blocker = None
            overlap_world = None
            overlap_bearing = None
            overlap_area_px = 0.0
            overlap_cy_norm = None
            if overlap_visible:
                ox_c, oy_c = overlap["centroid"]
                overlap_world = (
                    cam_x + ox_c * cs - oy_c * sn,
                    cam_y + ox_c * sn + oy_c * cs,
                )
                overlap_blocker = self._camera_blocker_report(p, overlap_world)
                overlap_image_err = -clamp(oy_c / half_width, -1.0, 1.0)
                overlap_bearing = self.p3.BEARING_SIGN * overlap_image_err * half_fov
                overlap_projection_distance = ox_c if ox_c > 0.0 else math.hypot(ox_c, oy_c)
                overlap_area_px = (
                    focal_px * focal_px * overlap["area_m2"] /
                    max(1e-6, overlap_projection_distance * overlap_projection_distance)
                )
                cy_span = max(1e-6, far_x - near_x)
                overlap_cy_norm = clamp(1.0 - (ox_c - near_x) / cy_span, 0.0, 1.0)
            targets.append({
                "id": "patch_{:02d}_{}".format(i + 1, p.color),
                "center": self._point_report(p.x, p.y),
                "distance_m": distance,
                "robot_frame_m": {"forward_x_m": robot_x, "left_y_m": robot_y},
                "camera_frame_m": {"forward_x_m": x_c, "left_y_m": y_c},
                "bearing_rad": bearing,
                "bearing_deg": math.degrees(bearing),
                "simulated_camera_bearing_rad": simulated_bearing,
                "simulated_camera_bearing_deg": math.degrees(simulated_bearing),
                "image_err": image_err,
                "camera_footprint_shape": "rectangle",
                "camera_rect_half_width_m": half_width,
                "camera_forward_range_m": {"min_m": near_x, "max_m": far_x},
                "in_front": x_c > 0.0,
                "inside_camera_rect_width": inside_width,
                "inside_camera_fov": inside_width,
                "inside_camera_min_range": x_c >= near_x,
                "inside_camera_max_range": x_c <= far_x,
                "inside_camera_range": inside_forward_range,
                "center_blocked_by_obstacle": center_blocker is not None,
                "center_blocking_obstacle": center_blocker,
                "blocked_by_obstacle": overlap_visible and overlap_blocker is not None,
                "blocking_obstacle": overlap_blocker,
                "estimated_area_px": area_px,
                "passes_min_area": area_px >= self.p3.MIN_AREA,
                "sim_camera_uses_overlap_centroid": True,
                "overlap_visible": overlap_visible,
                "overlap_area_m2": overlap["area_m2"],
                "overlap_ratio_of_patch": overlap["ratio"],
                "overlap_polygon_camera_frame_m": [
                    {"forward_x_m": x, "left_y_m": y}
                    for x, y in overlap["polygon"]
                ],
                "overlap_centroid_camera_frame_m": (
                    {
                        "forward_x_m": overlap["centroid"][0],
                        "left_y_m": overlap["centroid"][1],
                    }
                    if overlap_visible else None
                ),
                "overlap_centroid": (
                    self._point_report(*overlap_world)
                    if overlap_world is not None else None
                ),
                "overlap_bearing_rad": overlap_bearing,
                "overlap_bearing_deg": (
                    math.degrees(overlap_bearing)
                    if overlap_bearing is not None else None
                ),
                "overlap_estimated_area_px": overlap_area_px,
                "overlap_cy_norm": overlap_cy_norm,
                "overlap_blocked_by_obstacle": overlap_visible and overlap_blocker is not None,
                "overlap_blocking_obstacle": overlap_blocker,
            })
        targets.sort(key=lambda item: item["distance_m"])
        return targets

    def _sensor_snapshot(self):
        try:
            scan = sensors.simulate_lidar_raw(
                self.world, self.p3, n_rays=self.n_rays, noise_m=0.0)
            pts = self.p3.lidar_points_to_xy(scan)
            lidar_count = int(len(pts))
            if lidar_count:
                lidar_min = float(np.min(np.linalg.norm(pts, axis=1)))
            else:
                lidar_min = None
        except Exception as exc:
            lidar_count = 0
            lidar_min = None
            scan = ([], [], [])
            pts = []
            lidar_error = repr(exc)
        else:
            lidar_error = None

        near_x, far_x = self._camera_forward_range()
        cam_res = sensors.simulate_camera(
            self.world, self.p3, self.target_color,
            cam_max=far_x, cam_far=far_x, cam_min=near_x,
            camera_height_m=self._camera_mount_height(),
            cam_width_m=self.camera_width, cam_depth_m=self.camera_depth,
            cam_rear_blind_m=self.camera_rear_blind,
            robot_body_length_m=self.robot_body_length,
            camera_forward_offset_m=self.camera_forward_offset,
            camera_left_offset_m=self.camera_left_offset)
        if len(cam_res) >= 7:
            vis, bearing, area_ratio, cy_norm, n_visible, target_x, target_y = cam_res
            target_xy = (
                {"x_m": target_x, "y_m": target_y}
                if target_x is not None and target_y is not None else None
            )
        else:
            vis, bearing, area_ratio, cy_norm, n_visible = cam_res
            target_xy = None
        all_color_detections = {}
        for c in self.p3.TARGET_SEQUENCE:
            res = sensors.simulate_camera(
                self.world, self.p3, c,
                cam_max=far_x, cam_far=far_x, cam_min=near_x,
                camera_height_m=self._camera_mount_height(),
                cam_width_m=self.camera_width, cam_depth_m=self.camera_depth,
                cam_rear_blind_m=self.camera_rear_blind,
                robot_body_length_m=self.robot_body_length,
                camera_forward_offset_m=self.camera_forward_offset,
                camera_left_offset_m=self.camera_left_offset)
            if len(res) >= 7 and res[0] and res[5] is not None and res[6] is not None:
                all_color_detections[c] = {
                    "visible": True,
                    "bearing_rad": res[1],
                    "area_ratio": res[2],
                    "cy_norm": res[3],
                    "n_blobs": res[4],
                    "target_center_robot_frame_m": {"x_m": res[5], "y_m": res[6]},
                }
        return {
            "lidar": {
                "raw_return_count": int(len(scan[0])),
                "project3_filtered_point_count": lidar_count,
                "nearest_project3_point_m": lidar_min,
                "error": lidar_error,
            },
            "camera": {
                "target_color_used_by_sim": self.target_color,
                "mount_height_m": self._camera_mount_height(),
                "robot_body_height_m": self.robot_height,
                "camera_lift_above_robot_m": self.camera_lift,
                "camera_offset_robot_frame_m": {
                    "forward_x_m": self.camera_forward_offset,
                    "left_y_m": self.camera_left_offset,
                },
                "footprint_shape": "rectangle",
                "recognition_basis": "any visible patch overlap; bearing uses visible overlap centroid",
                "min_area_filter_applied_in_sim": False,
                "ground_visible_range_m": {"min_m": near_x, "max_m": far_x},
                "rect_forward_range_m": {"min_m": near_x, "max_m": far_x},
                "rear_blind_from_robot_back_m": self.camera_rear_blind,
                "rect_width_m": self.camera_width,
                    "rect_depth_m": self.camera_depth,
                    "rect_half_width_m": self._camera_rect_half_width(),
                    "HFOV_DEG_used_for_bearing": self.p3.HFOV_DEG,
                    "project3_camera_matrix_source": "practice/camera_2/practice10.py scaled to CAM_W x CAM_H",
                    "project3_camera_matrix_px": [
                        [getattr(self.p3, "CAMERA_FX_PX", None), 0.0, getattr(self.p3, "CAMERA_CX_PX", None)],
                        [0.0, getattr(self.p3, "CAMERA_FY_PX", None), getattr(self.p3, "CAMERA_CY_PX", None)],
                        [0.0, 0.0, 1.0],
                    ],
                    "visible": bool(vis),
                "bearing_rad": bearing,
                "bearing_deg": math.degrees(bearing),
                "area_ratio": area_ratio,
                "cy_norm": cy_norm,
                "visible_candidate_count": n_visible,
                "target_center_robot_frame_m": target_xy,
                "all_color_detections": all_color_detections,
            },
        }

    def _diagnostics(self, sensor_snapshot, nearest_obstacle, target_geometry):
        reasons = []
        required = self.world.current_target_color()
        current_target_patches = [p for p in self.world.patches if p.color == required]
        left, right = self.world.robot.wheel_contacts(self.bridge.ino["WHEEL_BASE"])

        def add(code, severity, message, evidence=None):
            reasons.append({
                "code": code,
                "severity": severity,
                "message": message,
                "evidence": evidence or {},
            })

        if self.world.finished:
            add("finished", "info", "All target colors were cleared in order.")
            return {"summary": "finished", "likely_reasons": reasons}

        missing = [color for color in ORDER if not any(p.color == color for p in self.world.patches)]
        if missing:
            add("missing_target_patches", "high",
                "Some required target color patches are not placed.",
                {"missing_colors": missing})

        if required and self.target_color != required:
            add("target_mismatch", "high",
                "The simulator camera target color differs from the judge's next required color.",
                {"camera_target": self.target_color, "judge_required_target": required})

        if self.sim_time >= 180.0:
            add("time_limit_reached", "high",
                "The 180 second project time limit has been reached without finishing.",
                {"sim_time_s": self.sim_time})

        if self.world.collisions > 0:
            add("collision_history", "high",
                "The robot has already collided during this run.",
                {"collisions": self.world.collisions})

        keepin = self._keepin_report()
        if keepin["enabled"] and keepin["current_center_margin_m"] is not None:
            margin = keepin["current_center_margin_m"]
            predicted = keepin["predicted_command_margin_m"]
            if margin < 0.0:
                add("outside_keepin_boundary", "high",
                    "The robot odometry pose is outside the 2.3m keep-in boundary.",
                    keepin)
            elif predicted is not None and predicted < 0.0:
                add("command_would_leave_keepin_boundary", "medium",
                    "The current command trajectory would leave the keep-in boundary.",
                    keepin)

        if nearest_obstacle is not None:
            clearance = nearest_obstacle["clearance_from_robot_body_m"]
            if clearance <= 1e-9:
                add("currently_intersecting_obstacle", "high",
                    "The robot body currently overlaps an obstacle.",
                    {"nearest_obstacle": nearest_obstacle})
            elif clearance < self.p3.COLLISION_DIST:
                add("inside_collision_margin", "medium",
                    "The nearest obstacle is inside the collision clearance used by project3.py.",
                    {
                        "nearest_obstacle": nearest_obstacle,
                        "COLLISION_DIST": self.p3.COLLISION_DIST,
                    })

        if self.tele.get("blocked"):
            add("dwa_blocked", "medium",
                "project3.py currently marks the DWA candidate path as blocked.",
                {
                    "clearance_score": self.tele.get("clr"),
                    "cmd_v": self.tele.get("v"),
                    "cmd_w": self.tele.get("w"),
                })

        camera = sensor_snapshot["camera"]
        if not camera["visible"]:
            geom = target_geometry.get(self.target_color, [])
            if not geom:
                add("camera_target_missing", "high",
                    "There is no patch for the color currently requested by the camera model.",
                    {"camera_target": self.target_color})
            else:
                nearest = geom[0]
                near_x, far_x = self._camera_forward_range()
                evidence = {"nearest_target": nearest,
                            "cam_min_m": near_x,
                            "cam_max_m": far_x,
                            "camera_footprint_shape": camera.get("footprint_shape"),
                            "camera_rect_half_width_m": self._camera_rect_half_width(),
                            "camera_height_m": self._camera_mount_height(),
                            "camera_offset_robot_frame_m": {
                                "forward_x_m": self.camera_forward_offset,
                                "left_y_m": self.camera_left_offset,
                            },
                            "HFOV_DEG_used_for_bearing": self.p3.HFOV_DEG,
                            "project3_py_MIN_AREA_for_real_camera_px": self.p3.MIN_AREA,
                            "min_area_filter_applied_in_sim": False}
                if not nearest.get("overlap_visible", False):
                    if not nearest["in_front"]:
                        msg = "No part of the nearest target patch overlaps the camera rectangle; its center is behind the robot."
                    elif not nearest["inside_camera_min_range"]:
                        msg = "No part of the nearest target patch overlaps the camera rectangle; it is mainly closer than the camera minimum forward range."
                    elif not nearest["inside_camera_max_range"]:
                        msg = "No part of the nearest target patch overlaps the camera rectangle; it is mainly beyond the camera maximum forward range."
                    elif not nearest["inside_camera_rect_width"]:
                        msg = "No part of the nearest target patch overlaps the camera rectangle width."
                    else:
                        msg = "No visible area of the nearest target patch overlaps the rectangular camera footprint."
                elif nearest["blocked_by_obstacle"]:
                    msg = "The visible target overlap is hidden by an obstacle that is taller than the camera sightline."
                else:
                    msg = "The nearest target patch has visible overlap, but the simulator camera did not report it; check occlusion and numeric geometry."
                add("camera_target_not_visible", "medium", msg, evidence)

        if required and current_target_patches:
            patches_with_wheels = [
                p for p in current_target_patches
                if p.contains(*left) and p.contains(*right)
            ]
            if not patches_with_wheels:
                add("not_in_scoring_region", "medium",
                    "Both wheel contact points are not inside the next required target patch.",
                    {
                        "required_target": required,
                        "left_wheel": self._point_report(*left),
                        "right_wheel": self._point_report(*right),
                    })
            elif abs(self.world.robot.v) >= 0.02:
                add("not_stopped_on_target", "medium",
                    "The robot is on the required target patch but has not stopped yet.",
                    {"robot_v_mps": self.world.robot.v, "stop_threshold_mps": 0.02})
            elif self.world.dwell < 1.0:
                add("dwell_time_incomplete", "low",
                    "The robot is stopped on the required patch but has not stayed for 1 second yet.",
                    {"dwell_s": self.world.dwell, "required_dwell_s": 1.0})

        if not reasons:
            add("not_finished_yet", "info",
                "No decisive failure condition is visible in this snapshot; inspect trajectory, command, and target geometry.")

        highest = "info"
        for severity in ("high", "medium", "low"):
            if any(r["severity"] == severity for r in reasons):
                highest = severity
                break
        return {"summary": "not_finished: highest_reason={}".format(highest),
                "likely_reasons": reasons}

    def _make_layout_report(self):
        rb = self.world.robot
        left, right = rb.wheel_contacts(self.bridge.ino["WHEEL_BASE"])
        near_x, far_x = self._camera_forward_range()
        sensor_snapshot = self._sensor_snapshot()
        nearest_obstacle = self._nearest_obstacle_snapshot()
        target_geometry = {color: self._target_geometry(color) for color in ORDER}
        return {
            "purpose": (
                "Complete simulator snapshot: use this to reproduce the scene, "
                "inspect why the robot could not pass, and compare project3.py / project3.ino behavior."
            ),
            "coordinate_system": {
                "world_m": "0..ARENA_M coordinates used internally by the simulator.",
                "centered_m": "world_m shifted by (-ARENA_M/2, -ARENA_M/2); robot default is (0,-1).",
                "robot_heading_theta": "+x 기준 CCW radians, same convention as project3.py.",
            },
            "world": {
                "arena_m": {"width_m": ARENA_M, "height_m": ARENA_M},
                "test_zone_m": {
                    "width_m": TEST_ZONE_M,
                    "height_m": TEST_ZONE_M,
                    "world_bounds_m": {
                        "x_min_m": TEST_ZONE_X,
                        "x_max_m": TEST_ZONE_X + TEST_ZONE_M,
                        "y_min_m": TEST_ZONE_Y,
                        "y_max_m": TEST_ZONE_Y + TEST_ZONE_M,
                    },
                    "centered_bounds_m": {
                        "x_min_m": TEST_ZONE_X - ARENA_M * 0.5,
                        "x_max_m": TEST_ZONE_X + TEST_ZONE_M - ARENA_M * 0.5,
                        "y_min_m": TEST_ZONE_Y - ARENA_M * 0.5,
                        "y_max_m": TEST_ZONE_Y + TEST_ZONE_M - ARENA_M * 0.5,
                    },
                },
                "keepin_boundary": self._keepin_report(),
                "outer_border_is_wall": self.world.walls_on,
            },
            "run_state": {
                "running": self.running,
                "paused": self.paused,
                "sim_time_s": self.sim_time,
                "sim_speed": self.sim_speed,
                "target_order": ORDER,
                "judge_next_required_color": self.world.current_target_color(),
                "camera_target_color": self.target_color,
                "cleared_by_order": dict(zip(ORDER, self.world.cleared)),
                "dwell_s": self.world.dwell,
                "collisions": self.world.collisions,
                "finished": self.world.finished,
            },
            "controller": self._controller_report(),
            "robot": {
                "pose": self._pose_report(rb.x, rb.y, rb.theta),
                "start_pose": self._pose_report(*self.start_pose),
                "collision_model": "rectangle",
                "body_footprint_m": {
                    "length_m": self.robot_body_length,
                    "width_m": self.robot_body_width,
                },
                "body_corners": [self._point_report(x, y) for x, y in self._robot_body_world_points()],
                "project3_dwa_radius_m": self.p3.ROBOT_RADIUS,
                "height_m": self.robot_height,
                "wheel_base_m": self.bridge.ino["WHEEL_BASE"],
                "wheel_contacts": {
                    "left": self._point_report(*left),
                    "right": self._point_report(*right),
                },
                "velocity": {"v_mps": rb.v, "w_radps": rb.w},
            },
            "objects": {
                "obstacles": [self._obstacle_report(o, i) for i, o in enumerate(self.world.obstacles)],
                "patches": [self._patch_report(p, i, left, right) for i, p in enumerate(self.world.patches)],
            },
            "simulator_parameters": {
                "n_rays": self.n_rays,
                "lidar_noise_m": self.lidar_noise,
                "cam_min_m": near_x,
                "cam_max_m": far_x,
                "camera_model": {
                    "mount_height_m": self._camera_mount_height(),
                    "robot_body_height_m": self.robot_height,
                    "camera_lift_above_robot_m": self.camera_lift,
                    "offset_robot_frame_m": {
                        "forward_x_m": self.camera_forward_offset,
                        "left_y_m": self.camera_left_offset,
                    },
                    "footprint_shape": "rectangle",
                    "recognition_basis": "any visible patch overlap; bearing uses visible overlap centroid",
                    "min_area_filter_applied_in_sim": False,
                    "project3_py_MIN_AREA_for_real_camera_px": self.p3.MIN_AREA,
                    "ground_visible_range_m": {"min_m": near_x, "max_m": far_x},
                    "rect_forward_range_m": {"min_m": near_x, "max_m": far_x},
                    "rear_blind_from_robot_back_m": self.camera_rear_blind,
                    "rect_width_m": self.camera_width,
                    "rect_depth_m": self.camera_depth,
                    "rect_half_width_m": self._camera_rect_half_width(),
                    "HFOV_DEG_used_for_bearing": self.p3.HFOV_DEG,
                    "project3_camera_matrix_source": "practice/camera_2/practice10.py scaled to CAM_W x CAM_H",
                    "project3_camera_matrix_px": [
                        [getattr(self.p3, "CAMERA_FX_PX", None), 0.0, getattr(self.p3, "CAMERA_CX_PX", None)],
                        [0.0, getattr(self.p3, "CAMERA_FY_PX", None), getattr(self.p3, "CAMERA_CY_PX", None)],
                        [0.0, 0.0, 1.0],
                    ],
                    "occlusion_uses_obstacle_height": True,
                },
                "obstacle_default_footprint_m": {"width_m": self.obstacle_w, "depth_m": self.obstacle_h},
                "obstacle_default_vertical_height_m": self.obstacle_z_h,
                "patch_default_size_m": self.patch_size,
            },
            "project3_py_parameters": {
                name: getattr(self.p3, name)
                for name in (
                    "CRUISE_V", "MAX_W", "W_RATE", "W_CLEAR", "W_GOAL",
                    "W_SPEED", "W_TURN", "W_SMOOTH",
                    "HORIZON_T", "COLLISION_DIST", "SLOW_DIST", "ROBOT_RADIUS",
                    "KEEPIN_ENABLED", "KEEPIN_SIZE_M", "KEEPIN_MARGIN_M",
                    "ODOM_WHEEL_R", "ODOM_WHEEL_BASE", "ODOM_PPR",
                    "ODOM_START_X", "ODOM_START_Y", "ODOM_START_TH", "ODOM_HOLD_S",
                    "MAX_RANGE_M", "V_MIN_RATIO", "V_SET_RATIOS", "V_SET_MIN", "V_SET",
                    "CLEAR_CAP", "GAP_BEARING_MAX", "GAP_BEARING_STEP",
                    "GAP_WINDOW_RAD", "GAP_MIN_CLEAR_MARGIN",
                    "GAP_PASS_LOOKAHEAD_M", "GAP_PASS_SIDE_MARGIN_M", "GAP_W_PASS",
                    "GAP_W_CLEAR", "GAP_W_TARGET", "GAP_W_TURN", "GAP_W_AWAY",
                    "AVOID_TRIGGER_MARGIN", "AVOID_RELEASE_MARGIN", "AVOID_LOST_MEMORY_S",
                    "ESCAPE_ROTATE_MARGIN", "ESCAPE_CREEP_MAX_V",
                    "ESCAPE_REVERSE_V", "ESCAPE_REVERSE_MIN_GAIN", "HFOV_DEG",
                    "DEFAULT_TARGET", "TARGET_SEQUENCE", "ARRIVE_CY",
                    "APPROACH_CY", "APPROACH_ALIGN_MAX", "APPROACH_STABLE_S",
                    "CLOSE_LOST_HOLD_CY",
                    "BEARING_SMOOTH_ALPHA",
                    "CLOSE_APPROACH_V", "CLOSE_APPROACH_MAX_S",
                    "BLIND_CREEP_V", "BLIND_CREEP_DIST_M",
                    "CREEP_STEER_GAIN", "CREEP_MAX_W", "DWELL_S",
                    "CENTER_TOL_M", "CENTER_MAX_V", "CENTER_SLOW_RADIUS_M",
                    "CENTER_GOAL_ALPHA", "CENTER_LOST_MEMORY_S", "CENTER_MAX_BEARING",
                    "SEARCH_W", "SEARCH_SWEEP_ANGLE_DEG",
                    "SEARCH_SWEEP_ANGLE_RAD", "SEARCH_SETTLE_RAD",
                    "SEARCH_SCAN_EDGE_HITS", "SEARCH_DRIVE_V", "SEARCH_DRIVE_S",
                    "SEARCH_SWEEP_S", "TARGET_LOST_MEMORY_S",
                    "EXPLORE_ENABLED", "EXPLORE_SPACING_M", "EXPLORE_EDGE_MARGIN_M",
                    "EXPLORE_REACH_DIST_M", "EXPLORE_SCAN_EDGE_HITS", "EXPLORE_MAX_V",
                    "EXPLORE_SLOW_DIST_M", "EXPLORE_TURN_IN_PLACE_RAD",
                    "EXPLORE_BLOCKED_S", "EXPLORE_STUCK_S", "EXPLORE_STUCK_DIST_M",
                    "COLOR_MEMORY_TTL_S", "WRONG_COLOR_AVOID_RADIUS_M",
                    "WRONG_COLOR_SCORE_PENALTY", "TARGET_MEMORY_REACHED_DIST_M",
                    "TARGET_MEMORY_MAX_V",
                    "BEARING_SIGN", "MIN_AREA", "LOOP_DT",
                    "PRACTICE10_FX_640", "PRACTICE10_FY_480",
                    "PRACTICE10_CX_640", "PRACTICE10_CY_480",
                    "CAMERA_FX_PX", "CAMERA_FY_PX", "CAMERA_CX_PX", "CAMERA_CY_PX",
                    "ROBOT_BODY_LENGTH_M", "ROBOT_BODY_WIDTH_M",
                    "CAMERA_FORWARD_OFFSET_M", "CAMERA_LEFT_OFFSET_M",
                    "CAMERA_REAR_BLIND_M", "CAMERA_RECT_WIDTH_M", "CAMERA_RECT_DEPTH_M",
                )
                if hasattr(self.p3, name)
            },
            "project3_ino_parameters": dict(self.bridge.ino),
            "telemetry": dict(self.tele),
            "run_history": self._run_history_report(),
            "sensor_snapshot": sensor_snapshot,
            "target_geometry": target_geometry,
            "nearest_obstacle": nearest_obstacle,
            "diagnostics": self._diagnostics(sensor_snapshot, nearest_obstacle, target_geometry),
        }

    def _controller_report(self):
        ctrl = self.ctrl

        def remaining(attr):
            value = getattr(ctrl, attr, 0.0)
            return max(0.0, value - self.sim_time) if value else 0.0

        last_seen_t = getattr(ctrl, "last_seen_t", -1e9)
        return {
            "seq_idx": getattr(ctrl, "seq_idx", None),
            "target_color": getattr(ctrl, "target_color", None),
            "done": getattr(ctrl, "done", False),
            "phase": getattr(ctrl, "phase", None),
            "last_w": getattr(ctrl, "last_w", None),
            "goal": getattr(ctrl, "goal", None),
            "clr": getattr(ctrl, "clr", None),
            "timers_abs_s": {
                "close_until": getattr(ctrl, "close_until", 0.0),
                "creep_until": getattr(ctrl, "creep_until", 0.0),
                "dwell_until": getattr(ctrl, "dwell_until", 0.0),
            },
            "timers_remaining_s": {
                "close": remaining("close_until"),
                "creep": remaining("creep_until"),
                "dwell": remaining("dwell_until"),
            },
            "last_seen": {
                "time_s": last_seen_t if last_seen_t > -1e8 else None,
                "age_s": self.sim_time - last_seen_t if last_seen_t > -1e8 else None,
                "bearing_rad": getattr(ctrl, "last_seen_bearing", None),
                "cy_norm": getattr(ctrl, "last_seen_cy", None),
                "smooth_bearing_rad": getattr(ctrl, "smooth_bearing", None),
                "search_dir": getattr(ctrl, "search_dir", None),
                "search_switch_t": getattr(ctrl, "search_switch_t", None),
                "search_anchor_theta_rad": getattr(ctrl, "search_anchor_theta", None),
                "search_anchor_theta_deg": (
                    math.degrees(getattr(ctrl, "search_anchor_theta"))
                    if getattr(ctrl, "search_anchor_theta", None) is not None else None
                ),
                "search_mode": getattr(ctrl, "search_mode", None),
                "search_edge_hits": getattr(ctrl, "search_edge_hits", None),
                "search_drive_until_s": getattr(ctrl, "search_drive_until", None),
                "search_drive_remaining_s": (
                    max(0.0, getattr(ctrl, "search_drive_until", 0.0) - self.sim_time)
                    if getattr(ctrl, "search_mode", None) == "DRIVE" else 0.0
                ),
            },
            "exploration": {
                "enabled": getattr(self.p3, "EXPLORE_ENABLED", None),
                "mode": getattr(ctrl, "search_mode", None),
                "target_idx": getattr(ctrl, "explore_target_idx", None),
                "target_world_m": (
                    {
                        "x_m": ctrl.explore_waypoints[ctrl.explore_target_idx][0],
                        "y_m": ctrl.explore_waypoints[ctrl.explore_target_idx][1],
                    }
                    if getattr(ctrl, "explore_target_idx", None) is not None
                    and 0 <= ctrl.explore_target_idx < len(getattr(ctrl, "explore_waypoints", []))
                    else None
                ),
                "visited_count": sum(1 for v in getattr(ctrl, "explore_visited", []) if v),
                "deferred_count": sum(1 for v in getattr(ctrl, "explore_deferred", []) if v),
                "waypoint_count": len(getattr(ctrl, "explore_waypoints", [])),
                "priority_count": getattr(ctrl, "explore_priority_count", None),
                "cycle": getattr(ctrl, "explore_cycle", None),
                "anchor_world_m": (
                    {
                        "x_m": getattr(ctrl, "explore_anchor_x", None),
                        "y_m": getattr(ctrl, "explore_anchor_y", None),
                    }
                    if getattr(ctrl, "explore_anchor_x", None) is not None else None
                ),
                "last_distance_m": getattr(ctrl, "explore_last_dist", None),
                "blocked_since_s": getattr(ctrl, "explore_blocked_since", None),
                "spacing_m": getattr(self.p3, "EXPLORE_SPACING_M", None),
                "reach_dist_m": getattr(self.p3, "EXPLORE_REACH_DIST_M", None),
            },
            "color_memory": dict(getattr(ctrl, "color_memory", {})),
            "avoidance": {
                "active": getattr(ctrl, "avoid_active", None),
                "gap_goal_rad": getattr(ctrl, "avoid_goal", None),
                "gap_goal_deg": (
                    math.degrees(getattr(ctrl, "avoid_goal"))
                    if getattr(ctrl, "avoid_goal", None) is not None else None
                ),
                "direct_clearance_m": getattr(ctrl, "avoid_direct_clr", None),
                "target_sector_clearance_m": getattr(ctrl, "avoid_target_sector_clear", None),
                "gap_sector_clearance_m": getattr(ctrl, "avoid_sector_clear", None),
                "gap_pass_clearance_m": getattr(ctrl, "avoid_pass_clear", None),
                "gap_required_width_m": (
                    2.0 * getattr(ctrl, "avoid_required_half_width")
                    if getattr(ctrl, "avoid_required_half_width", None) is not None else None
                ),
                "gap_pass_lookahead_m": getattr(ctrl, "avoid_lookahead", None),
                "gap_score": getattr(ctrl, "avoid_score", None),
                "trigger_margin_m": getattr(self.p3, "AVOID_TRIGGER_MARGIN", None),
                "release_margin_m": getattr(self.p3, "AVOID_RELEASE_MARGIN", None),
            },
            "approach": {
                "align_since_s": getattr(ctrl, "approach_align_since", None),
                "aligned_for_s": (
                    self.sim_time - getattr(ctrl, "approach_align_since", 0.0)
                    if getattr(ctrl, "approach_align_since", -1.0) >= 0.0 else 0.0
                ),
                "ready": getattr(ctrl, "approach_ready", None),
                "close_peak_cy": getattr(ctrl, "close_peak_cy", None),
                "close_lost_hold_cy": getattr(self.p3, "CLOSE_LOST_HOLD_CY", None),
            },
            "center_alignment": {
                "goal_world_m": (
                    {
                        "x_m": getattr(ctrl, "center_goal_x", None),
                        "y_m": getattr(ctrl, "center_goal_y", None),
                    }
                    if getattr(ctrl, "center_goal_x", None) is not None else None
                ),
                "last_error_robot_frame_m": (
                    {
                        "x_m": getattr(ctrl, "last_target_x", None),
                        "y_m": getattr(ctrl, "last_target_y", None),
                        "distance_m": getattr(ctrl, "last_center_dist", None),
                    }
                    if getattr(ctrl, "last_center_dist", None) is not None else None
                ),
                "tolerance_m": getattr(self.p3, "CENTER_TOL_M", None),
                "max_v_mps": getattr(self.p3, "CENTER_MAX_V", None),
                "lost_memory_s": getattr(self.p3, "CENTER_LOST_MEMORY_S", None),
            },
        }

    def view_scale(self):
        return SCALE * self.view_zoom

    def w2s(self, wx, wy):
        s = self.view_scale()
        cx = MARGIN + ARENA_PX * 0.5
        cy = MARGIN + ARENA_PX * 0.5
        return (cx + (wx - self.view_cx) * s, cy - (wy - self.view_cy) * s)

    def s2w(self, sx, sy):
        s = self.view_scale()
        cx = MARGIN + ARENA_PX * 0.5
        cy = MARGIN + ARENA_PX * 0.5
        return (self.view_cx + (sx - cx) / s, self.view_cy - (sy - cy) / s)

    def in_arena(self, pos):
        return MARGIN <= pos[0] <= MARGIN + ARENA_PX and MARGIN <= pos[1] <= MARGIN + ARENA_PX

    def zoom_at(self, wheel_y, pos):
        before = self.s2w(*pos)
        factor = 1.15 ** wheel_y
        self.view_zoom = clamp(self.view_zoom * factor, MIN_VIEW_ZOOM, MAX_VIEW_ZOOM)
        after = self.s2w(*pos)
        self.view_cx += before[0] - after[0]
        self.view_cy += before[1] - after[1]

    def start_pan(self, pos):
        if self.in_arena(pos):
            self.view_panning = True
            self.view_pan_last = pos

    def pan_view(self, pos):
        if not self.view_panning or self.view_pan_last is None:
            return
        dx = pos[0] - self.view_pan_last[0]
        dy = pos[1] - self.view_pan_last[1]
        s = self.view_scale()
        self.view_cx -= dx / s
        self.view_cy += dy / s
        self.view_pan_last = pos

    def end_pan(self):
        self.view_panning = False
        self.view_pan_last = None

    def _param_panel_top(self):
        return MARGIN + ui(188)

    def _param_viewport_rect(self):
        top = self._param_panel_top()
        return pygame.Rect(
            PANEL_X - ui(4),
            top,
            PANEL_W,
            max(0, WIN_H - top - ui(4)),
        )

    def _param_content_height(self):
        return sum(ui(30) if sl is None else ui(44) for _, sl in self.sliders)

    def _param_max_scroll(self):
        view = self._param_viewport_rect()
        return max(0, self._param_content_height() - view.h)

    def _clamp_param_scroll(self):
        self.param_scroll = clamp(self.param_scroll, 0, self._param_max_scroll())

    def _param_scrollbar_rects(self):
        view = self._param_viewport_rect()
        content_h = self._param_content_height()
        max_scroll = max(0, content_h - view.h)
        if max_scroll <= 0 or view.h <= ui(32):
            return None
        track = pygame.Rect(PANEL_X + PANEL_W - ui(10), view.y, ui(6), view.h)
        thumb_h = max(ui(24), int(round(track.h * view.h / max(1, content_h))))
        thumb_h = min(track.h, thumb_h)
        travel = max(1, track.h - thumb_h)
        thumb_y = track.y + int(round((self.param_scroll / max_scroll) * travel))
        thumb = pygame.Rect(track.x, thumb_y, track.w, thumb_h)
        return track, thumb, max_scroll

    def _set_scroll_from_thumb_y(self, y):
        info = self._param_scrollbar_rects()
        if info is None:
            return
        track, thumb, max_scroll = info
        travel = max(1, track.h - thumb.h)
        thumb_y = clamp(y - self.param_scroll_drag_offset, track.y, track.y + travel)
        self.param_scroll = clamp((thumb_y - track.y) / travel * max_scroll, 0, max_scroll)

    def start_param_scroll_drag(self, pos):
        info = self._param_scrollbar_rects()
        if info is None:
            return False
        track, thumb, _ = info
        hit = track.inflate(ui(8), ui(4)).collidepoint(pos)
        if not hit:
            return False
        self.param_scroll_drag = True
        if thumb.collidepoint(pos):
            self.param_scroll_drag_offset = pos[1] - thumb.y
        else:
            self.param_scroll_drag_offset = thumb.h // 2
            self._set_scroll_from_thumb_y(pos[1])
        return True

    def drag_param_scroll(self, pos):
        if self.param_scroll_drag:
            self._set_scroll_from_thumb_y(pos[1])
            return True
        return False

    def end_param_scroll_drag(self):
        self.param_scroll_drag = False
        self.param_scroll_drag_offset = 0

    def brain_tick(self, commit):
        """결정/상태는 전부 p3.Controller.tick() 에 위임(단일 진실원).
           sim 은 합성 센서·시각화 글루만 담당. py 의 tick() 을 고치면 여기로 자동 반영."""
        p3 = self.p3
        color = self.ctrl.target_color
        if color is not None:
            self.target_color = color

        scan = sensors.simulate_lidar_raw(self.world, p3, n_rays=self.n_rays,
                                          noise_m=self.lidar_noise)
        pts = p3.lidar_points_to_xy(scan)
        if len(pts):
            lidar_min = float(np.min(np.linalg.norm(pts, axis=1)))
        else:
            lidar_min = None
        if color is None:
            seen, bearing, ar, cy, ncnt = False, 0.0, 0.0, 0.0, 0
            target_xy = None
            color_detections = {}
        else:
            near_x, far_x = self._camera_forward_range()
            color_detections = {}
            cam_results = {}
            for c in p3.TARGET_SEQUENCE:
                res = sensors.simulate_camera(
                    self.world, p3, c,
                    cam_max=far_x, cam_far=far_x, cam_min=near_x,
                    camera_height_m=self._camera_mount_height(),
                    cam_width_m=self.camera_width, cam_depth_m=self.camera_depth,
                    cam_rear_blind_m=self.camera_rear_blind,
                    robot_body_length_m=self.robot_body_length,
                    camera_forward_offset_m=self.camera_forward_offset,
                    camera_left_offset_m=self.camera_left_offset)
                cam_results[c] = res
                if len(res) >= 7 and res[0] and res[5] is not None and res[6] is not None:
                    color_detections[c] = {
                        "visible": True,
                        "bearing_rad": res[1],
                        "area_ratio": res[2],
                        "cy_norm": res[3],
                        "n_blobs": res[4],
                        "target_center_robot_frame_m": {"x_m": res[5], "y_m": res[6]},
                    }
            cam_res = cam_results.get(color, (False, 0.0, 0.0, 0.0, 0, None, None))
            if len(cam_res) >= 7:
                vis, bearing, ar, cy, ncnt, target_x, target_y = cam_res[:7]
                target_xy = (target_x, target_y) if target_x is not None and target_y is not None else None
            else:
                vis, bearing, ar, cy, ncnt = cam_res
                target_xy = None
            seen = vis

        ctrl = self.ctrl if commit else copy.deepcopy(self.ctrl)
        v, w, state = ctrl.tick(pts, seen, bearing, cy, self.sim_time,
                                pose=self._p3_pose_tuple(), target_xy=target_xy,
                                color_detections=color_detections)
        if commit:
            p3.send_vw(self.link, v, w)

        self._update_viz(pts, v, w)
        if seen and state in ("SEEK", "CENTER", "CENTER_BLIND", "CLOSE", "AVOID", "BLOCKED"):
            d = 0.6
            self.bear_pt = (self.world.robot.x + d * math.cos(self.world.robot.theta + bearing),
                            self.world.robot.y + d * math.sin(self.world.robot.theta + bearing))
        else:
            self.bear_pt = None
        self.tele.update(v=v, w=w, gb=ctrl.goal, clr=ctrl.clr, blocked=(state == "BLOCKED"),
                         see=seen, npts=len(pts), ar=ar, bstate=state,
                         avoid_active=getattr(ctrl, "avoid_active", False),
                         avoid_goal=getattr(ctrl, "avoid_goal", 0.0),
                         avoid_direct_clr=getattr(ctrl, "avoid_direct_clr", None),
                         avoid_target_sector_clear=getattr(ctrl, "avoid_target_sector_clear", None),
                         avoid_sector_clear=getattr(ctrl, "avoid_sector_clear", None),
                         avoid_pass_clear=getattr(ctrl, "avoid_pass_clear", None),
                         avoid_score=getattr(ctrl, "avoid_score", None))
        self.last_brain_snapshot = {
            "t_s": self.sim_time,
            "target_color": color,
            "state": state,
            "camera": {
                "visible": bool(seen),
                "bearing_rad": bearing,
                "area_ratio": ar,
                "cy_norm": cy,
                "visible_candidate_count": ncnt,
                "target_center_robot_frame_m": (
                    {"x_m": target_xy[0], "y_m": target_xy[1]}
                    if target_xy is not None else None
                ),
                "all_color_detections": color_detections,
            },
            "lidar": {
                "raw_return_count": int(len(scan[0])),
                "project3_filtered_point_count": int(len(pts)),
                "nearest_project3_point_m": lidar_min,
            },
            "avoidance": {
                "active": getattr(ctrl, "avoid_active", False),
                "gap_goal_rad": getattr(ctrl, "avoid_goal", 0.0),
                "direct_clearance_m": getattr(ctrl, "avoid_direct_clr", None),
                "target_sector_clearance_m": getattr(ctrl, "avoid_target_sector_clear", None),
                "gap_sector_clearance_m": getattr(ctrl, "avoid_sector_clear", None),
                "gap_pass_clearance_m": getattr(ctrl, "avoid_pass_clear", None),
                "gap_required_width_m": (
                    2.0 * getattr(ctrl, "avoid_required_half_width")
                    if getattr(ctrl, "avoid_required_half_width", None) is not None else None
                ),
                "gap_pass_lookahead_m": getattr(ctrl, "avoid_lookahead", None),
                "gap_score": getattr(ctrl, "avoid_score", None),
            },
        }

    def _update_viz(self, pts, v, w):
        rb = self.world.robot
        cs, sn = math.cos(rb.theta), math.sin(rb.theta)
        if len(pts):
            wx = rb.x + pts[:, 0] * cs - pts[:, 1] * sn
            wy = rb.y + pts[:, 0] * sn + pts[:, 1] * cs
            self.pts_world = np.column_stack((wx, wy))
        else:
            self.pts_world = np.empty((0, 2))
        xs, ys = self.p3.predict_xy(max(v, 0.05), w)
        tx = rb.x + xs * cs - ys * sn
        ty = rb.y + xs * sn + ys * cs
        self.traj_world = np.column_stack((tx, ty))

    def step(self, dt_real):
        if any(self.bridge.check_reload()):
            self.p3 = self.bridge.p3
            self.arduino.ino = self.bridge.ino
            self.world.set_robot_radius(self.p3.ROBOT_RADIUS)
            self._sync_project3_geometry()
            new_ctrl = self.p3.Controller()
            new_ctrl.__dict__.update(self.ctrl.__dict__)
            self.ctrl = new_ctrl
            self.sliders = self._make_sliders()
            self._say("RELOADED code")

        if self.running and not self.paused:
            self.acc += dt_real * self.sim_speed
            self.acc = min(self.acc, 0.25)
            ino = self.bridge.ino
            while self.acc >= self.SIM_DT:
                self.brain_acc += self.SIM_DT
                if self.brain_acc >= self.p3.LOOP_DT:
                    self.brain_acc -= self.p3.LOOP_DT
                    self.brain_tick(commit=True)
                self.arduino.update(self.SIM_DT)
                self.world.step_physics(self.arduino.wL, self.arduino.wR,
                                        ino["WHEEL_R"], ino["WHEEL_BASE"], self.SIM_DT)
                self.sim_time += self.SIM_DT
                self._append_history_sample("physics")
                self.acc -= self.SIM_DT
        else:
            self.brain_tick(commit=False)

    def on_click(self, pos):
        for b in self.tool_buttons + self.action_buttons:
            if b.click(pos):
                return
        if self.sign_button.click(pos):
            return
        if self.in_arena(pos):
            wx, wy = self.s2w(*pos)
            if self.tool == "Obstacle":
                self.obstacle_placing = True
                self.obstacle_anchor = (wx, wy)
                self.obstacle_preview = Obstacle(
                    wx, wy, self.obstacle_w, self.obstacle_h, 0.0, self.obstacle_z_h)
            elif self.tool in PATCH_COL:
                self.world.patches.append(Patch(wx, wy, self.tool, self.patch_size))
                self._history_event("place_patch", {
                    "x_m": wx, "y_m": wy, "color": self.tool, "size_m": self.patch_size,
                })
            elif self.tool == "Robot":
                self.world.robot.x, self.world.robot.y = wx, wy
                self.robot_placing = True
                self._history_event("place_robot", {
                    "x_m": wx, "y_m": wy, "theta_rad": self.world.robot.theta,
                })
            elif self.tool == "Erase":
                self._erase_near(wx, wy)

    def _erase_near(self, wx, wy):
        best = None
        bestd = 0.18
        for o in self.world.obstacles:
            d = o.distance_to_point(wx, wy)
            if d < bestd:
                best, bestd = ("o", o), d
        for p in self.world.patches:
            d = math.hypot(p.x - wx, p.y - wy)
            if d < bestd:
                best, bestd = ("p", p), d
        if best:
            details = {"kind": "obstacle" if best[0] == "o" else "patch"}
            if best[0] == "o":
                details.update({
                    "x_m": best[1].x, "y_m": best[1].y,
                    "width_m": best[1].w, "depth_m": best[1].h,
                    "theta_rad": best[1].theta, "z_h_m": best[1].z_h,
                })
            else:
                details.update({
                    "x_m": best[1].x, "y_m": best[1].y,
                    "color": best[1].color, "size_m": best[1].size,
                })
            (self.world.obstacles if best[0] == "o" else self.world.patches).remove(best[1])
            self._history_event("erase", details)

    def on_mouse_up(self, pos):
        if self.obstacle_placing:
            self._update_obstacle_preview(pos)
            if self.obstacle_preview is not None:
                p = self.obstacle_preview
                self.world.obstacles.append(Obstacle(p.x, p.y, p.w, p.h, p.theta, p.z_h))
                self._history_event("place_obstacle", {
                    "x_m": p.x, "y_m": p.y,
                    "width_m": p.w, "depth_m": p.h,
                    "theta_rad": p.theta, "theta_deg": math.degrees(p.theta),
                    "z_h_m": p.z_h,
                })
            self.obstacle_placing = False
            self.obstacle_anchor = None
            self.obstacle_preview = None
        self.robot_placing = False

    def on_motion(self, pos):
        if self.obstacle_placing:
            self._update_obstacle_preview(pos)
        elif self.robot_placing and self.in_arena(pos):
            wx, wy = self.s2w(*pos)
            dx, dy = wx - self.world.robot.x, wy - self.world.robot.y
            if math.hypot(dx, dy) > 0.03:
                self.world.robot.theta = math.atan2(dy, dx)

    def _update_obstacle_preview(self, pos):
        if self.obstacle_anchor is None:
            return
        ax, ay = self.obstacle_anchor
        theta = 0.0
        if self.in_arena(pos):
            wx, wy = self.s2w(*pos)
            dx, dy = wx - ax, wy - ay
            if math.hypot(dx, dy) > 0.03:
                theta = math.atan2(dy, dx)
        elif self.obstacle_preview is not None:
            theta = self.obstacle_preview.theta
        self.obstacle_preview = Obstacle(
            ax, ay, self.obstacle_w, self.obstacle_h, theta, self.obstacle_z_h)

    def on_wheel(self, y, pos):
        if pos[0] >= PANEL_X:
            self.param_scroll = clamp(self.param_scroll - y * ui(24), 0, self._param_max_scroll())
        elif self.in_arena(pos):
            self.zoom_at(y, pos)

    def draw(self, surf, font, bigfont):
        surf.fill(BG)
        arena = pygame.Rect(MARGIN, MARGIN, ARENA_PX, ARENA_PX)
        pygame.draw.rect(surf, ARENA_BG, arena)
        surf.set_clip(arena)
        view_s = self.view_scale()
        zx, zy = self.w2s(TEST_ZONE_X, TEST_ZONE_Y + TEST_ZONE_M)
        zw = TEST_ZONE_M * view_s
        zh = TEST_ZONE_M * view_s
        zone_surf = pygame.Surface((max(1, int(zw)), max(1, int(zh))), pygame.SRCALPHA)
        zone_surf.fill((*TEST_ZONE_BG, 150))
        surf.blit(zone_surf, (zx, zy))
        pygame.draw.rect(surf, TEST_ZONE_EDGE, (zx, zy, zw, zh), max(1, ui(2)))
        if getattr(self.p3, "KEEPIN_ENABLED", False):
            kb = self._keepin_bounds_world()
            kx, ky = self.w2s(kb["x_min_m"], kb["y_max_m"])
            kw = (kb["x_max_m"] - kb["x_min_m"]) * view_s
            kh = (kb["y_max_m"] - kb["y_min_m"]) * view_s
            pygame.draw.rect(surf, KEEPIN_EDGE, (kx, ky, kw, kh), max(1, ui(2)))
        for i in range(1, int(ARENA_M / 0.5)):
            gx = i * 0.5
            pygame.draw.line(surf, GRID, self.w2s(gx, 0.0), self.w2s(gx, ARENA_M))
            gy = i * 0.5
            pygame.draw.line(surf, GRID, self.w2s(0.0, gy), self.w2s(ARENA_M, gy))

        for p in self.world.patches:
            cx, cy = self.w2s(p.x, p.y)
            spx = p.size * view_s
            s = pygame.Surface((max(1, int(spx)), max(1, int(spx))), pygame.SRCALPHA)
            s.fill((*PATCH_COL[p.color], 130))
            surf.blit(s, (cx - spx / 2, cy - spx / 2))
            pygame.draw.rect(surf, PATCH_COL[p.color],
                             (cx - spx / 2, cy - spx / 2, spx, spx), 1)

        for o in self.world.obstacles:
            self._draw_obstacle(surf, o)
        if self.obstacle_preview is not None:
            self._draw_obstacle(
                surf, self.obstacle_preview,
                fill=(80, 110, 96), edge=(170, 215, 190), alpha=150)

        self._draw_camera_footprint(surf)
        self._draw_robot(surf)
        self._draw_overlays(surf)
        surf.set_clip(None)
        pygame.draw.rect(surf, WALL, arena, 1)
        self._draw_panel(surf, font, bigfont)
        self._draw_telemetry(surf, font)

    def _draw_obstacle(self, surf, obstacle, fill=OBST, edge=OBST_EDGE, alpha=255):
        pts = [(int(round(x)), int(round(y))) for x, y in
               (self.w2s(cx, cy) for cx, cy in obstacle.corners())]
        center = tuple(int(round(v)) for v in self.w2s(obstacle.x, obstacle.y))
        tip = tuple(int(round(v)) for v in self.w2s(*obstacle.local_to_world(obstacle.half_w, 0.0)))
        width = max(1, ui(1))
        if alpha < 255:
            layer = pygame.Surface((WIN_W, WIN_H), pygame.SRCALPHA)
            pygame.draw.polygon(layer, (*fill, alpha), pts)
            pygame.draw.polygon(layer, (*edge, min(255, alpha + 60)), pts, width)
            pygame.draw.line(layer, (*edge, min(255, alpha + 80)), center, tip, max(1, ui(2)))
            surf.blit(layer, (0, 0))
        else:
            pygame.draw.polygon(surf, fill, pts)
            pygame.draw.polygon(surf, edge, pts, width)
            pygame.draw.line(surf, edge, center, tip, max(1, ui(2)))

    def _draw_camera_footprint(self, surf):
        pts = [self.w2s(x, y) for x, y in self._camera_rect_world_points()]
        pts = [(int(round(x)), int(round(y))) for x, y in pts]
        if len(pts) < 3:
            return
        layer = pygame.Surface((WIN_W, WIN_H), pygame.SRCALPHA)
        pygame.draw.polygon(layer, CAM_AREA_C, pts)
        surf.blit(layer, (0, 0))

    def _alpha_circle(self, surf, center, rpx, rgba):
        rpx = int(rpx)
        if rpx < 1:
            return
        s = pygame.Surface((rpx * 2, rpx * 2), pygame.SRCALPHA)
        pygame.draw.circle(s, rgba, (rpx, rpx), rpx)
        surf.blit(s, (center[0] - rpx, center[1] - rpx))

    def _draw_robot(self, surf):
        rb = self.world.robot
        cx, cy = self.w2s(rb.x, rb.y)
        c = (int(cx), int(cy))
        view_s = self.view_scale()
        self._alpha_circle(surf, c, self.p3.SLOW_DIST * view_s, (220, 200, 70, 36))
        self._alpha_circle(surf, c, self.p3.COLLISION_DIST * view_s, (230, 80, 80, 60))
        body_pts = [(int(round(x)), int(round(y)))
                    for x, y in (self.w2s(wx, wy) for wx, wy in self._robot_body_world_points())]
        pygame.draw.polygon(surf, ROBOT_C, body_pts)
        pygame.draw.polygon(surf, (20, 30, 28), body_pts, 2)
        cam_x, cam_y = self._camera_world_pose()
        cam_p = self.w2s(cam_x, cam_y)
        pygame.draw.circle(surf, (40, 70, 72), (int(cam_p[0]), int(cam_p[1])), max(3, ui(4)))
        hx = rb.x + self.robot_body_length * 0.75 * math.cos(rb.theta)
        hy = rb.y + self.robot_body_length * 0.75 * math.sin(rb.theta)
        pygame.draw.line(surf, HEAD_C, c, self.w2s(hx, hy), 2)
        wb = self.bridge.ino["WHEEL_BASE"]
        left, right = rb.wheel_contacts(wb)
        for wc in (left, right):
            p = self.w2s(*wc)
            pygame.draw.circle(surf, (250, 250, 255), (int(p[0]), int(p[1])), 3)

    def _draw_overlays(self, surf):
        rb = self.world.robot
        pts = [(int(round(x)), int(round(y)))
               for x, y in (self.w2s(wx, wy) for wx, wy in self._camera_rect_world_points())]
        pygame.draw.lines(surf, FOV_C, True, pts, 1)
        arena = pygame.Rect(MARGIN, MARGIN, ARENA_PX, ARENA_PX)
        for px, py in self.pts_world:
            sp = self.w2s(px, py)
            ip = (int(sp[0]), int(sp[1]))
            if arena.collidepoint(ip):
                surf.set_at(ip, LIDAR_C)
                pygame.draw.circle(surf, LIDAR_C, ip, 2)
        if len(self.traj_world) > 1:
            pts = [self.w2s(x, y) for x, y in self.traj_world]
            pygame.draw.lines(surf, TRAJ_C, False, pts, 2)
        if self.bear_pt:
            pygame.draw.line(surf, BEAR_C, self.w2s(rb.x, rb.y),
                             self.w2s(*self.bear_pt), 2)

    def _draw_panel(self, surf, font, bigfont):
        for b in self.tool_buttons + self.action_buttons:
            b.draw(surf, font)
        y = MARGIN + ui(150)
        self.sign_button.rect = pygame.Rect(PANEL_X, y, ui(200), ui(28))
        self.sign_button.label = "BEARING_SIGN: {:+.0f}".format(self.p3.BEARING_SIGN)
        self.sign_button.draw(surf, font)
        tgt = bigfont.render("target: " + self.target_color, True,
                             PATCH_COL[self.target_color])
        surf.blit(tgt, (PANEL_X + ui(210), y + ui(2)))

        self._clamp_param_scroll()
        top = self._param_panel_top()
        clip = self._param_viewport_rect()
        surf.set_clip(clip)
        y = top - self.param_scroll
        mouse_pos = pygame.mouse.get_pos()
        tooltip = None
        slider_w = PANEL_W - ui(24)
        for label, sl in self.sliders:
            if sl is None:
                hdr = font.render(label, True, (130, 200, 180))
                if top - ui(20) < y < WIN_H:
                    surf.blit(hdr, (PANEL_X, y + ui(6)))
                y += ui(30)
            else:
                if top - ui(30) < y < WIN_H:
                    sl.draw(surf, font, PANEL_X, y, slider_w)
                    if tooltip is None:
                        tooltip = sl.hover_tip(mouse_pos)
                y += ui(44)
        surf.set_clip(None)
        self._draw_param_scrollbar(surf)
        self._draw_tooltip(surf, font, tooltip, mouse_pos)

    def _draw_param_scrollbar(self, surf):
        info = self._param_scrollbar_rects()
        if info is None:
            return
        track, thumb, _ = info
        pygame.draw.rect(surf, (38, 42, 52), track, border_radius=max(2, ui(3)))
        hover = thumb.inflate(ui(8), ui(4)).collidepoint(pygame.mouse.get_pos())
        col = (132, 146, 170) if (hover or self.param_scroll_drag) else (86, 96, 118)
        pygame.draw.rect(surf, col, thumb, border_radius=max(2, ui(3)))

    def _draw_tooltip(self, surf, font, text, pos):
        if not text:
            return
        pad_x = ui(9)
        pad_y = ui(7)
        rendered = font.render(text, True, TXT)
        w = rendered.get_width() + 2 * pad_x
        h = rendered.get_height() + 2 * pad_y
        x = pos[0] + ui(14)
        y = pos[1] + ui(16)
        x = min(x, WIN_W - w - ui(6))
        y = min(y, WIN_H - h - ui(6))
        x = max(ui(6), x)
        y = max(ui(6), y)
        box = pygame.Rect(x, y, w, h)
        layer = pygame.Surface((w, h), pygame.SRCALPHA)
        pygame.draw.rect(layer, (10, 12, 16, 225), layer.get_rect(), border_radius=ui(5))
        pygame.draw.rect(layer, (100, 112, 135, 210), layer.get_rect(), 1, border_radius=ui(5))
        layer.blit(rendered, (pad_x, pad_y))
        surf.blit(layer, box)

    def _draw_telemetry(self, surf, font):
        t = self.tele
        st = "RUN" if (self.running and not self.paused) else ("PAUSE" if self.paused else "STOP")
        cleared = "".join(
            (c[0] if self.world.cleared[i] else c[0].lower())
            for i, c in enumerate(ORDER))
        lines = [
            f"{st}  t={self.sim_time:5.1f}s  speed x{self.sim_speed:.2f}",
            f"brain={self.target_color}[{t['bstate']}]  judge[{cleared}]  jdwell={self.world.dwell:.2f}",
            f"see={'Y' if t['see'] else 'n'}  gb={t['gb']:+.2f}rad  ar={t['ar']:.3f}",
            f"cmd v={t['v']:+.3f} w={t['w']:+.2f}  clr={t['clr']:.2f}  {'BLOCKED' if t['blocked'] else ''}",
            f"odom=({self._p3_pose_tuple()[0]:+.2f},{self._p3_pose_tuple()[1]:+.2f})",
            f"collisions={self.world.collisions}   {'*** FINISHED ***' if self.world.finished else ''}",
            f"tool={self.tool}",
        ]
        rendered = [font.render(ln, True, TXT) for ln in lines]
        pad_x = ui(8)
        pad_y = ui(6)
        line_h = max(font.get_linesize(), ui(18))
        bg_w = max((r.get_width() for r in rendered), default=ui(300)) + 2 * pad_x
        bg_h = line_h * len(lines) + 2 * pad_y
        bg = pygame.Surface((bg_w, bg_h), pygame.SRCALPHA)
        bg.fill((10, 12, 16, 170))
        surf.blit(bg, (MARGIN + ui(6), MARGIN + ui(6)))
        for i, rendered_line in enumerate(rendered):
            surf.blit(
                rendered_line,
                (MARGIN + ui(6) + pad_x, MARGIN + ui(6) + pad_y + i * line_h),
            )
        msg, _ = self.toast
        if msg:
            m = font.render(msg, True, (250, 230, 120))
            surf.blit(m, (MARGIN + ARENA_PX - m.get_width() - ui(10), MARGIN + ui(10)))


def main():
    selftest = "--selftest" in sys.argv
    if selftest:
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    pygame.init()
    configure_layout(WIN_W, WIN_H)
    screen = pygame.display.set_mode((WIN_W, WIN_H), pygame.RESIZABLE)
    pygame.display.set_caption("project3 drive_sim")
    font, bigfont = make_fonts()
    clock = pygame.time.Clock()

    sim = Simulator()
    frames = 0
    run = True
    while run:
        dt = clock.tick(60) / 1000.0
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                run = False
            elif ev.type == pygame.VIDEORESIZE or ev.type == getattr(pygame, "WINDOWRESIZED", None):
                w = getattr(ev, "w", getattr(ev, "x", WIN_W))
                h = getattr(ev, "h", getattr(ev, "y", WIN_H))
                surface = pygame.display.get_surface()
                if surface is not None:
                    screen = surface
                sim.resize(w, h)
                font, bigfont = make_fonts()
            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                consumed = False
                if sim.start_param_scroll_drag(ev.pos):
                    consumed = True
                if not consumed:
                    for _, sl in sim.sliders:
                        if sl and sl.handle(ev):
                            consumed = True
                            break
                if not consumed:
                    sim.on_click(ev.pos)
            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 2:
                sim.start_pan(ev.pos)
            elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
                sim.end_param_scroll_drag()
                for _, sl in sim.sliders:
                    if sl:
                        sl.handle(ev)
                sim.on_mouse_up(ev.pos)
            elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 2:
                sim.end_pan()
            elif ev.type == pygame.MOUSEMOTION:
                if sim.param_scroll_drag:
                    sim.drag_param_scroll(ev.pos)
                elif sim.view_panning:
                    sim.pan_view(ev.pos)
                else:
                    handled = False
                    for _, sl in sim.sliders:
                        if sl and sl.drag and sl.handle(ev):
                            handled = True
                            break
                    if not handled:
                        sim.on_motion(ev.pos)
            elif ev.type == pygame.MOUSEWHEEL:
                sim.on_wheel(ev.y, pygame.mouse.get_pos())
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_SPACE:
                    sim.start() if not sim.running else sim.toggle_pause()
                elif ev.key == pygame.K_r:
                    sim.reset()
                elif ev.key in (pygame.K_1, pygame.K_2, pygame.K_3):
                    sim.ctrl.seq_idx = ev.key - pygame.K_1
                    if hasattr(sim.ctrl, "_reset_for_next_target"):
                        sim.ctrl._reset_for_next_target()
                    else:
                        sim.ctrl.dwell_until = 0.0
                    sim.target_color = sim.p3.TARGET_SEQUENCE[sim.ctrl.seq_idx]

        m, tleft = sim.toast
        if tleft > 0:
            sim.toast = (m, tleft - dt)
        else:
            sim.toast = ("", 0.0)

        sim.step(dt)
        sim.draw(screen, font, bigfont)
        pygame.display.flip()

        frames += 1
        if selftest and frames > 30:
            run = False

    pygame.quit()
    if selftest:
        print("[selftest] OK: 30 frames rendered, sim_time={:.2f}".format(sim.sim_time))


if __name__ == "__main__":
    main()


