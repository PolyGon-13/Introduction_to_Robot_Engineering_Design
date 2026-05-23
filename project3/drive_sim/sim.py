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

import json
import math
import os
import sys
from datetime import datetime

import numpy as np
import pygame

from code_bridge import CodeBridge, recompute_derived, PROJECT3_DIR
from arduino_model import ArduinoSim
from world import World, Obstacle, Patch, ARENA_M, TEST_ZONE_M, TEST_ZONE_X, TEST_ZONE_Y, ORDER
import sensors

# ----------------------------------------------------------------- 레이아웃
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
SCALE = ARENA_PX / ARENA_M            # px per meter
LAYOUT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "layout.json")

# 색
BG = (24, 26, 32)
ARENA_BG = (40, 43, 52)
TEST_ZONE_BG = (50, 62, 54)
TEST_ZONE_EDGE = (110, 180, 130)
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
BEAR_C = (230, 110, 220)
PATCH_COL = {"RED": (210, 70, 70), "YELLOW": (220, 200, 70), "BLUE": (80, 120, 220)}
BTN = (60, 64, 78)
BTN_ON = (70, 130, 110)
BTN_HL = (84, 90, 108)


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


def make_fonts():
    font_size = max(10, int(round(BASE_FONT_SIZE * UI_SCALE)))
    big_size = max(12, int(round(BASE_BIGFONT_SIZE * UI_SCALE)))
    return (
        pygame.font.SysFont("dejavusansmono,monospace", font_size),
        pygame.font.SysFont("dejavusans,sans", big_size, bold=True),
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


# ----------------------------------------------------------------- 위젯
class Button:
    def __init__(self, rect, label, cb, is_on=None):
        self.rect = pygame.Rect(rect)
        self.label = label
        self.cb = cb
        self.is_on = is_on            # 함수: 활성 상태면 True

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
    def __init__(self, label, get, set_, lo, hi, is_int=False, fmt="{:.2f}"):
        self.label = label
        self.get = get
        self.set = set_
        self.lo = lo
        self.hi = hi
        self.is_int = is_int
        self.fmt = fmt
        self.rect = pygame.Rect(0, 0, 0, 0)   # 매 프레임 갱신
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
        surf.blit(lab, (x, y))
        val = font.render(self.fmt.format(self.get()), True, TXT)
        surf.blit(val, (x + w - val.get_width(), y))
        pygame.draw.rect(surf, (60, 64, 78), self.rect, border_radius=max(2, ui(3)))
        hx = self._to_px()
        pygame.draw.circle(surf, (160, 200, 230), (hx, self.rect.centery), ui(7))

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


# ----------------------------------------------------------------- 시뮬레이터
class Simulator:
    SIM_DT = 0.005

    def __init__(self):
        self.bridge = CodeBridge()
        self.p3 = self.bridge.p3
        self.arduino = ArduinoSim(self.bridge.ino)
        self.world = World()
        self.world.set_robot_radius(self.p3.ROBOT_RADIUS)

        # 실행 상태
        self.running = False
        self.paused = False
        self.start_pose = (self.world.robot.x, self.world.robot.y, self.world.robot.theta)

        # 브레인 상태
        self.last_w = 0.0
        self.brain_acc = 0.0
        self.acc = 0.0
        self.sim_time = 0.0
        self.target_color = self.p3.DEFAULT_TARGET

        # sim 파라미터
        self.sim_speed = 1.0
        self.n_rays = 360
        self.lidar_noise = 0.0
        self.cam_max = 2.5
        self.obstacle_w = 0.30
        self.obstacle_h = 0.15
        self.patch_size = 0.25

        # 월드 보기 상태: 마우스 휠로 아레나를 확대/축소한다.
        self.view_zoom = 1.0
        self.view_cx = ARENA_M * 0.5
        self.view_cy = ARENA_M * 0.5
        self.view_panning = False
        self.view_pan_last = None

        # 텔레메트리/시각화
        self.tele = dict(v=0.0, w=0.0, gb=0.0, clr=0.0, blocked=False,
                         see=False, npts=0, ar=0.0)
        self.pts_world = np.empty((0, 2))
        self.traj_world = np.empty((0, 2))
        self.bear_pt = None

        # 입력 도구
        self.tool = "Obstacle"
        self.robot_placing = False
        self.obstacle_placing = False
        self.obstacle_anchor = None
        self.obstacle_preview = None
        self.param_scroll = 0
        self.toast = ("", 0.0)

        # 시리얼 링크(실제 send_vw 가 호출)
        sim = self

        class _Link:
            def write(self, data):
                return sim.arduino.write(data)
        self.link = _Link()

        self._build_ui()

    def resize(self, window_w, window_h):
        configure_layout(window_w, window_h)
        self.param_scroll = clamp(self.param_scroll, 0, ui(1400))
        self._build_ui()

    # ----- UI 구성 -----
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

        # BEARING_SIGN 토글
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

        S = Slider
        return [
            ("— DWA / robot (project3.py) —", None),
            ("CRUISE_V", S("CRUISE_V", p3get("CRUISE_V"), p3set("CRUISE_V"), 0.05, 0.30)),
            ("MAX_W", S("MAX_W", p3get("MAX_W"), p3set("MAX_W"), 0.3, 2.0)),
            ("W_RATE", S("W_RATE", p3get("W_RATE"), p3set("W_RATE"), 0.05, 1.0)),
            ("W_CLEAR", S("W_CLEAR", p3get("W_CLEAR"), p3set("W_CLEAR"), 0.0, 3.0)),
            ("W_GOAL", S("W_GOAL", p3get("W_GOAL"), p3set("W_GOAL"), 0.0, 3.0)),
            ("HORIZON_T", S("HORIZON_T", p3get("HORIZON_T"), p3set("HORIZON_T"), 0.4, 2.5)),
            ("COLLISION_DIST", S("COLLISION_DIST", p3get("COLLISION_DIST"), p3set("COLLISION_DIST"), 0.10, 0.40)),
            ("SLOW_DIST", S("SLOW_DIST", p3get("SLOW_DIST"), p3set("SLOW_DIST"), 0.15, 0.80)),
            ("ROBOT_RADIUS", S("ROBOT_RADIUS", p3get("ROBOT_RADIUS"), p3set("ROBOT_RADIUS"), 0.08, 0.20)),
            ("MAX_RANGE_M", S("MAX_RANGE_M", p3get("MAX_RANGE_M"), p3set("MAX_RANGE_M"), 0.5, 4.0)),
            ("V_MIN_RATIO", S("V_MIN_RATIO", p3get("V_MIN_RATIO"), p3set("V_MIN_RATIO"), 0.1, 1.0)),
            ("CLEAR_CAP", S("CLEAR_CAP", p3get("CLEAR_CAP"), p3set("CLEAR_CAP"), 0.2, 1.5)),
            ("HFOV_DEG", S("HFOV_DEG", p3get("HFOV_DEG"), p3set("HFOV_DEG"), 30.0, 120.0)),
            ("— Arduino (project3.ino) —", None),
            ("wheel tau", S("wheel tau", ardget("tau"), ardset("tau"), 0.02, 0.40)),
            ("wheel accel", S("wheel accel", ardget("wheel_accel_max"), ardset("wheel_accel_max"), 5.0, 100.0)),
            ("— Simulator —", None),
            ("sim speed", S("sim speed", simget("sim_speed"), simset("sim_speed"), 0.25, 3.0)),
            ("lidar noise", S("lidar noise", simget("lidar_noise"), simset("lidar_noise"), 0.0, 0.03, fmt="{:.3f}")),
            ("n_rays", S("n_rays", simget("n_rays"), simset("n_rays"), 90, 720, is_int=True, fmt="{:.0f}")),
            ("cam_max", S("cam_max", simget("cam_max"), simset("cam_max"), 1.0, 3.0)),
            ("obstacle w", S("obstacle w", simget("obstacle_w"), simset("obstacle_w"), 0.05, 0.80)),
            ("obstacle h", S("obstacle h", simget("obstacle_h"), simset("obstacle_h"), 0.05, 0.50)),
            ("patch size", S("patch size", simget("patch_size"), simset("patch_size"), 0.10, 0.50)),
        ]

    # ----- 액션 -----
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
        self.running = True
        self.paused = False
        self._say("RUN")

    def toggle_pause(self):
        if self.running:
            self.paused = not self.paused

    def reset(self):
        self.running = False
        self.paused = False
        x, y, th = self.start_pose
        self.world.reset_run(x, y, th)
        self.arduino.V_cmd = self.arduino.W_cmd = 0.0
        self.arduino.wL = self.arduino.wR = 0.0
        self.last_w = 0.0
        self.brain_acc = self.acc = 0.0
        self.sim_time = 0.0
        self.target_color = self.p3.DEFAULT_TARGET
        self._say("RESET")

    def clear(self):
        self.world.obstacles.clear()
        self.world.patches.clear()
        self._say("CLEARED")

    def save_layout(self):
        data = self._make_save_data()
        with open(LAYOUT_PATH, "w") as f:
            json.dump(data, f, indent=2, ensure_ascii=False, default=json_default)
        self._say("SAVED layout.json")

    def _make_save_data(self):
        data = {
            "schema": "drive_sim_layout_v2",
            "saved_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "note": (
                "robot/obstacles/patches are the compact loadable layout. "
                "report contains the full simulator context and failure diagnostics "
                "for Codex/Claude-style code analysis."
            ),
            "robot": [self.world.robot.x, self.world.robot.y, self.world.robot.theta],
            "obstacles": [[o.x, o.y, o.w, o.h, o.theta] for o in self.world.obstacles],
            "patches": [[p.x, p.y, p.color, p.size] for p in self.world.patches],
            "report": self._make_layout_report(),
        }
        return data

    def load_layout(self):
        if not os.path.exists(LAYOUT_PATH):
            self._say("no layout.json")
            return
        with open(LAYOUT_PATH) as f:
            d = json.load(f)
        rb = d["robot"]
        self.world.robot.x, self.world.robot.y, self.world.robot.theta = rb
        self.world.obstacles = [self._load_obstacle(o) for o in d.get("obstacles", [])]
        self.world.patches = [self._load_patch(p) for p in d.get("patches", [])]
        self.start_pose = tuple(rb)
        self._say("LOADED layout.json")

    def _load_obstacle(self, data):
        if isinstance(data, dict):
            center = data.get("center_m", data)
            size = data.get("size_m", data)
            x = center.get("x_m", center.get("x", 0.0))
            y = center.get("y_m", center.get("y", 0.0))
            w = size.get("width_m", size.get("w_m", data.get("w", self.obstacle_w)))
            h = size.get("height_m", size.get("h_m", data.get("h", self.obstacle_h)))
            theta = data.get("theta_rad", math.radians(data.get("theta_deg", 0.0)))
            return Obstacle(x, y, w, h, theta)
        if len(data) >= 5:
            return Obstacle(data[0], data[1], data[2], data[3], data[4])
        if len(data) >= 4:
            return Obstacle(data[0], data[1], data[2], data[3])
        if len(data) == 3:
            # 구버전 원형 장애물 저장값 [x, y, r]은 같은 지름의 정사각형으로 변환.
            return Obstacle(data[0], data[1], data[2] * 2.0, data[2] * 2.0)
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
        body_clearance = o.distance_to_point(rb.x, rb.y) - self.p3.ROBOT_RADIUS
        corners = [self._point_report(x, y) for x, y in o.corners()]
        return {
            "id": "obstacle_{:02d}".format(idx + 1),
            "type": "rotated_rectangle",
            "center": self._point_report(o.x, o.y),
            "size_m": {"width_m": o.w, "height_m": o.h},
            "theta_rad": o.theta,
            "theta_deg": math.degrees(o.theta),
            "corners": corners,
            "distance_to_robot_center_m": o.distance_to_point(rb.x, rb.y),
            "clearance_from_robot_body_m": body_clearance,
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
            clearance = d_center - self.p3.ROBOT_RADIUS
            item = {
                "id": "obstacle_{:02d}".format(i + 1),
                "distance_to_robot_center_m": d_center,
                "clearance_from_robot_body_m": clearance,
                "center": self._point_report(o.x, o.y),
                "theta_deg": math.degrees(o.theta),
                "size_m": {"width_m": o.w, "height_m": o.h},
            }
            if best is None or clearance < best["clearance_from_robot_body_m"]:
                best = item
        return best

    def _target_geometry(self, color):
        rb = self.world.robot
        cs, sn = math.cos(rb.theta), math.sin(rb.theta)
        half_fov = self.p3.HALF_HFOV_RAD
        focal_px = (self.p3.CAM_W * 0.5) / max(1e-6, math.tan(half_fov))
        targets = []
        for i, p in enumerate(self.world.patches):
            if p.color != color:
                continue
            rx, ry = p.x - rb.x, p.y - rb.y
            x_r = rx * cs + ry * sn
            y_r = -rx * sn + ry * cs
            distance = math.hypot(x_r, y_r)
            bearing = math.atan2(y_r, x_r) if distance > 1e-9 else 0.0
            side_px = focal_px * p.size / max(1e-6, distance)
            area_px = side_px * side_px
            targets.append({
                "id": "patch_{:02d}_{}".format(i + 1, p.color),
                "center": self._point_report(p.x, p.y),
                "distance_m": distance,
                "robot_frame_m": {"forward_x_m": x_r, "left_y_m": y_r},
                "bearing_rad": bearing,
                "bearing_deg": math.degrees(bearing),
                "in_front": x_r > 0.0,
                "inside_camera_fov": abs(bearing) <= half_fov,
                "inside_camera_range": distance <= self.cam_max,
                "estimated_area_px": area_px,
                "passes_min_area": area_px >= self.p3.MIN_AREA,
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

        vis, bearing, area_ratio, cy_norm, n_visible = sensors.simulate_camera(
            self.world, self.p3, self.target_color, cam_max=self.cam_max)
        return {
            "lidar": {
                "raw_return_count": int(len(scan[0])),
                "project3_filtered_point_count": lidar_count,
                "nearest_project3_point_m": lidar_min,
                "error": lidar_error,
            },
            "camera": {
                "target_color_used_by_sim": self.target_color,
                "visible": bool(vis),
                "bearing_rad": bearing,
                "bearing_deg": math.degrees(bearing),
                "area_ratio": area_ratio,
                "cy_norm": cy_norm,
                "visible_candidate_count": n_visible,
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

        if nearest_obstacle is not None:
            clearance = nearest_obstacle["clearance_from_robot_body_m"]
            if clearance < 0.0:
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
                evidence = {"nearest_target": nearest, "cam_max_m": self.cam_max,
                            "HALF_HFOV_DEG": math.degrees(self.p3.HALF_HFOV_RAD),
                            "MIN_AREA": self.p3.MIN_AREA}
                if not nearest["in_front"]:
                    msg = "The nearest target patch is behind the robot."
                elif not nearest["inside_camera_fov"]:
                    msg = "The nearest target patch is outside the camera horizontal FOV."
                elif not nearest["inside_camera_range"]:
                    msg = "The nearest target patch is beyond the simulator camera range."
                elif not nearest["passes_min_area"]:
                    msg = "The nearest target patch appears too small for project3.py MIN_AREA."
                else:
                    msg = "The camera model did not report the target despite a placed patch."
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
            "robot": {
                "pose": self._pose_report(rb.x, rb.y, rb.theta),
                "start_pose": self._pose_report(*self.start_pose),
                "radius_m": self.p3.ROBOT_RADIUS,
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
                "cam_max_m": self.cam_max,
                "obstacle_default_size_m": {"width_m": self.obstacle_w, "height_m": self.obstacle_h},
                "patch_default_size_m": self.patch_size,
            },
            "project3_py_parameters": {
                name: getattr(self.p3, name)
                for name in (
                    "CRUISE_V", "MAX_W", "W_RATE", "W_CLEAR", "W_GOAL",
                    "HORIZON_T", "COLLISION_DIST", "SLOW_DIST", "ROBOT_RADIUS",
                    "MAX_RANGE_M", "V_MIN_RATIO", "CLEAR_CAP", "HFOV_DEG",
                    "DEFAULT_TARGET", "BEARING_SIGN", "MIN_AREA", "LOOP_DT",
                )
                if hasattr(self.p3, name)
            },
            "project3_ino_parameters": dict(self.bridge.ino),
            "telemetry": dict(self.tele),
            "sensor_snapshot": sensor_snapshot,
            "target_geometry": target_geometry,
            "nearest_obstacle": nearest_obstacle,
            "diagnostics": self._diagnostics(sensor_snapshot, nearest_obstacle, target_geometry),
        }

    # ----- 좌표 변환 -----
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

    # ----- 브레인 1틱 (main() 글루 미러) -----
    def brain_tick(self, commit):
        p3 = self.p3
        scan = sensors.simulate_lidar_raw(self.world, p3, n_rays=self.n_rays,
                                          noise_m=self.lidar_noise)
        pts = p3.lidar_points_to_xy(scan)
        vis, bearing, ar, cy, ncnt = sensors.simulate_camera(
            self.world, p3, self.target_color, cam_max=self.cam_max)
        seen = vis
        goal = bearing if seen else p3.GOAL_BEARING_FALLBACK
        v, w, clr, blocked = p3.choose_cmd(pts, goal, self.last_w)
        if commit:
            p3.send_vw(self.link, v, w)
            self.last_w = w
        # 시각화 좌표 변환
        self._update_viz(pts, v, w)
        if seen:
            d = 0.6
            self.bear_pt = (self.world.robot.x + d * math.cos(self.world.robot.theta + bearing),
                            self.world.robot.y + d * math.sin(self.world.robot.theta + bearing))
        else:
            self.bear_pt = None
        self.tele.update(v=v, w=w, gb=goal, clr=clr, blocked=blocked,
                         see=seen, npts=len(pts), ar=ar)

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

    # ----- 한 프레임 스텝 -----
    def step(self, dt_real):
        if any(self.bridge.check_reload()):
            self.p3 = self.bridge.p3
            self.arduino.ino = self.bridge.ino
            self.world.set_robot_radius(self.p3.ROBOT_RADIUS)
            self.last_w = 0.0
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
                self.acc -= self.SIM_DT
        else:
            self.brain_tick(commit=False)   # 정지 중에도 미리보기

    # ----- 입력 -----
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
                self.obstacle_preview = Obstacle(wx, wy, self.obstacle_w, self.obstacle_h, 0.0)
            elif self.tool in PATCH_COL:
                self.world.patches.append(Patch(wx, wy, self.tool, self.patch_size))
            elif self.tool == "Robot":
                self.world.robot.x, self.world.robot.y = wx, wy
                self.robot_placing = True
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
            (self.world.obstacles if best[0] == "o" else self.world.patches).remove(best[1])

    def on_mouse_up(self, pos):
        if self.obstacle_placing:
            self._update_obstacle_preview(pos)
            if self.obstacle_preview is not None:
                p = self.obstacle_preview
                self.world.obstacles.append(Obstacle(p.x, p.y, p.w, p.h, p.theta))
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
        self.obstacle_preview = Obstacle(ax, ay, self.obstacle_w, self.obstacle_h, theta)

    def on_wheel(self, y, pos):
        if pos[0] >= PANEL_X:
            self.param_scroll = clamp(self.param_scroll - y * ui(24), 0, ui(1400))
        elif self.in_arena(pos):
            self.zoom_at(y, pos)

    # ----- 렌더 -----
    def draw(self, surf, font, bigfont):
        surf.fill(BG)
        arena = pygame.Rect(MARGIN, MARGIN, ARENA_PX, ARENA_PX)
        pygame.draw.rect(surf, ARENA_BG, arena)
        surf.set_clip(arena)
        view_s = self.view_scale()
        # 시험 배치 영역: 장애물/색상 영역이 놓이는 약 2m x 2m 구역.
        zx, zy = self.w2s(TEST_ZONE_X, TEST_ZONE_Y + TEST_ZONE_M)
        zw = TEST_ZONE_M * view_s
        zh = TEST_ZONE_M * view_s
        zone_surf = pygame.Surface((max(1, int(zw)), max(1, int(zh))), pygame.SRCALPHA)
        zone_surf.fill((*TEST_ZONE_BG, 150))
        surf.blit(zone_surf, (zx, zy))
        pygame.draw.rect(surf, TEST_ZONE_EDGE, (zx, zy, zw, zh), max(1, ui(2)))
        # 그리드
        for i in range(1, int(ARENA_M / 0.5)):
            gx = i * 0.5
            pygame.draw.line(surf, GRID, self.w2s(gx, 0.0), self.w2s(gx, ARENA_M))
            gy = i * 0.5
            pygame.draw.line(surf, GRID, self.w2s(0.0, gy), self.w2s(ARENA_M, gy))

        # 패치(반투명)
        for p in self.world.patches:
            cx, cy = self.w2s(p.x, p.y)
            spx = p.size * view_s
            s = pygame.Surface((max(1, int(spx)), max(1, int(spx))), pygame.SRCALPHA)
            s.fill((*PATCH_COL[p.color], 130))
            surf.blit(s, (cx - spx / 2, cy - spx / 2))
            pygame.draw.rect(surf, PATCH_COL[p.color],
                             (cx - spx / 2, cy - spx / 2, spx, spx), 1)

        # 장애물
        for o in self.world.obstacles:
            self._draw_obstacle(surf, o)
        if self.obstacle_preview is not None:
            self._draw_obstacle(
                surf, self.obstacle_preview,
                fill=(80, 110, 96), edge=(170, 215, 190), alpha=150)

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
        # 안전영역(반투명): SLOW 먼저, COLLISION 위에
        self._alpha_circle(surf, c, self.p3.SLOW_DIST * view_s, (220, 200, 70, 36))
        self._alpha_circle(surf, c, self.p3.COLLISION_DIST * view_s, (230, 80, 80, 60))
        # 몸체
        pygame.draw.circle(surf, ROBOT_C, c, int(self.p3.ROBOT_RADIUS * view_s))
        pygame.draw.circle(surf, (20, 30, 28), c, int(self.p3.ROBOT_RADIUS * view_s), 2)
        # 헤딩
        hx = rb.x + self.p3.ROBOT_RADIUS * 1.6 * math.cos(rb.theta)
        hy = rb.y + self.p3.ROBOT_RADIUS * 1.6 * math.sin(rb.theta)
        pygame.draw.line(surf, HEAD_C, c, self.w2s(hx, hy), 2)
        # 바퀴 접점
        wb = self.bridge.ino["WHEEL_BASE"]
        left, right = rb.wheel_contacts(wb)
        for wc in (left, right):
            p = self.w2s(*wc)
            pygame.draw.circle(surf, (250, 250, 255), (int(p[0]), int(p[1])), 3)

    def _draw_overlays(self, surf):
        rb = self.world.robot
        # 카메라 FOV
        half = self.p3.HALF_HFOV_RAD
        for s in (-1, 1):
            a = rb.theta + s * half
            ex = rb.x + self.cam_max * math.cos(a)
            ey = rb.y + self.cam_max * math.sin(a)
            pygame.draw.line(surf, FOV_C, self.w2s(rb.x, rb.y), self.w2s(ex, ey), 1)
        # 라이다 점
        arena = pygame.Rect(MARGIN, MARGIN, ARENA_PX, ARENA_PX)
        for px, py in self.pts_world:
            sp = self.w2s(px, py)
            ip = (int(sp[0]), int(sp[1]))
            if arena.collidepoint(ip):
                surf.set_at(ip, LIDAR_C)
                pygame.draw.circle(surf, LIDAR_C, ip, 2)
        # DWA 선택 궤적
        if len(self.traj_world) > 1:
            pts = [self.w2s(x, y) for x, y in self.traj_world]
            pygame.draw.lines(surf, TRAJ_C, False, pts, 2)
        # 타깃 방위선
        if self.bear_pt:
            pygame.draw.line(surf, BEAR_C, self.w2s(rb.x, rb.y),
                             self.w2s(*self.bear_pt), 2)

    def _draw_panel(self, surf, font, bigfont):
        for b in self.tool_buttons + self.action_buttons:
            b.draw(surf, font)
        # BEARING_SIGN 토글
        y = MARGIN + ui(150)
        self.sign_button.rect = pygame.Rect(PANEL_X, y, ui(200), ui(28))
        self.sign_button.label = "BEARING_SIGN: {:+.0f}".format(self.p3.BEARING_SIGN)
        self.sign_button.draw(surf, font)
        tgt = bigfont.render("target: " + self.target_color, True,
                             PATCH_COL[self.target_color])
        surf.blit(tgt, (PANEL_X + ui(210), y + ui(2)))

        # 슬라이더(스크롤 영역)
        top = MARGIN + ui(188)
        clip = pygame.Rect(PANEL_X - ui(4), top, PANEL_W, WIN_H - top - ui(4))
        surf.set_clip(clip)
        y = top - self.param_scroll
        for label, sl in self.sliders:
            if sl is None:
                hdr = font.render(label, True, (130, 200, 180))
                if top - ui(20) < y < WIN_H:
                    surf.blit(hdr, (PANEL_X, y + ui(6)))
                y += ui(30)
            else:
                if top - ui(30) < y < WIN_H:
                    sl.draw(surf, font, PANEL_X, y, PANEL_W - ui(12))
                y += ui(44)
        surf.set_clip(None)

    def _draw_telemetry(self, surf, font):
        t = self.tele
        st = "RUN" if (self.running and not self.paused) else ("PAUSE" if self.paused else "STOP")
        cleared = "".join(
            (c[0] if self.world.cleared[i] else c[0].lower())
            for i, c in enumerate(ORDER))
        lines = [
            f"{st}  t={self.sim_time:5.1f}s  speed x{self.sim_speed:.2f}",
            f"target={self.target_color}  order[{cleared}]  dwell={self.world.dwell:.2f}",
            f"see={'Y' if t['see'] else 'n'}  gb={t['gb']:+.2f}rad  ar={t['ar']:.3f}",
            f"cmd v={t['v']:+.3f} w={t['w']:+.2f}  clr={t['clr']:.2f}  {'BLOCKED' if t['blocked'] else ''}",
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
                for _, sl in sim.sliders:
                    if sl and sl.handle(ev):
                        consumed = True
                        break
                if not consumed:
                    sim.on_click(ev.pos)
            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 2:
                sim.start_pan(ev.pos)
            elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
                for _, sl in sim.sliders:
                    if sl:
                        sl.handle(ev)
                sim.on_mouse_up(ev.pos)
            elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 2:
                sim.end_pan()
            elif ev.type == pygame.MOUSEMOTION:
                if sim.view_panning:
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
                    sim.target_color = ORDER[ev.key - pygame.K_1]

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
