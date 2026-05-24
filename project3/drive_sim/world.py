# -*- coding: utf-8 -*-
"""
시뮬레이션 월드: 로봇 자세, 장애물, 색상영역(색종이), 차동구동 물리, 심판.

좌표계(실제 코드와 동일): x=전방, y=좌측(+), w>0=좌회전(CCW).
월드는 4m×4m 표시 평면. 중앙 2m×2m가 시험 배치 영역이며,
테두리는 벽이 아니라 열린 공간으로 취급한다.
로봇 자세 (x, y, theta) [m, m, rad].
장애물은 2D 발자국(w/h)과 수직 높이(z_h)를 함께 가진다.
"""

import math

ARENA_M = 4.0
TEST_ZONE_M = 2.0
TEST_ZONE_X = (ARENA_M - TEST_ZONE_M) * 0.5
TEST_ZONE_Y = (ARENA_M - TEST_ZONE_M) * 0.5
ORDER = ["RED", "YELLOW", "BLUE"]   # 통과 순서
DEFAULT_OBSTACLE_Z_H = 0.23         # 장애물 수직 높이 23cm


class Obstacle:
    def __init__(self, x, y, w=0.24, h=0.12, theta=0.0, z_h=DEFAULT_OBSTACLE_Z_H):
        self.x = x
        self.y = y
        self.w = w          # 발자국 폭 (m) — 로컬 x축 방향
        self.h = h          # 발자국 깊이 (m) — 로컬 y축 방향
        self.theta = theta  # +x 기준 CCW 회전각 (rad)
        self.z_h = z_h      # 수직 높이 (m)

    @property
    def half_w(self):
        return self.w * 0.5

    @property
    def half_h(self):
        return self.h * 0.5

    def world_to_local(self, px, py):
        dx = px - self.x
        dy = py - self.y
        c = math.cos(self.theta)
        s = math.sin(self.theta)
        return c * dx + s * dy, -s * dx + c * dy

    def local_to_world(self, lx, ly):
        c = math.cos(self.theta)
        s = math.sin(self.theta)
        return self.x + c * lx - s * ly, self.y + s * lx + c * ly

    def corners(self):
        hw, hh = self.half_w, self.half_h
        return [
            self.local_to_world(-hw, -hh),
            self.local_to_world(hw, -hh),
            self.local_to_world(hw, hh),
            self.local_to_world(-hw, hh),
        ]

    def contains(self, px, py):
        lx, ly = self.world_to_local(px, py)
        return -self.half_w <= lx <= self.half_w and -self.half_h <= ly <= self.half_h

    def distance_to_point(self, px, py):
        lx, ly = self.world_to_local(px, py)
        dx = max(abs(lx) - self.half_w, 0.0)
        dy = max(abs(ly) - self.half_h, 0.0)
        return math.hypot(dx, dy)


class Patch:
    """바닥 색종이(축 정렬 사각형). 라이다는 못 보고 카메라만 본다."""
    def __init__(self, x, y, color, size=0.25):
        self.x = x          # 중심
        self.y = y
        self.color = color  # "RED"/"YELLOW"/"BLUE"
        self.size = size    # 한 변 길이 (m)

    def contains(self, px, py):
        h = self.size * 0.5
        return (self.x - h <= px <= self.x + h) and (self.y - h <= py <= self.y + h)


class Robot:
    def __init__(self, x=2.0, y=1.0, theta=math.pi / 2):
        self.x = x
        self.y = y
        self.theta = theta   # +x 기준 CCW
        self.v = 0.0         # 현재 실제 선속도(텔레메트리)
        self.w = 0.0

    def wheel_contacts(self, wheel_base):
        """좌/우 바퀴 바닥 접점 (월드 좌표). 좌측 = heading 기준 +90°."""
        hb = wheel_base * 0.5
        lx = -math.sin(self.theta)   # 좌측 단위벡터
        ly = math.cos(self.theta)
        left = (self.x + hb * lx, self.y + hb * ly)
        right = (self.x - hb * lx, self.y - hb * ly)
        return left, right


class World:
    def __init__(self):
        self.robot = Robot()
        self.obstacles = []
        self.patches = []
        self.walls_on = False

        # 심판 상태
        self.collisions = 0
        self._colliding = False
        self.target_idx = 0          # 현재 통과해야 할 색 인덱스(ORDER)
        self.dwell = 0.0             # 영역 내 정지 유지 시간
        self.cleared = [False, False, False]
        self.finished = False

    # ---- 배치/초기화 ----
    def current_target_color(self):
        if self.target_idx < len(ORDER):
            return ORDER[self.target_idx]
        return None

    def reset_run(self, x=None, y=None, theta=None):
        if x is not None:
            self.robot.x, self.robot.y, self.robot.theta = x, y, theta
        self.robot.v = self.robot.w = 0.0
        self.collisions = 0
        self._colliding = False
        self.target_idx = 0
        self.dwell = 0.0
        self.cleared = [False, False, False]
        self.finished = False

    # ---- 물리 적분(정확한 차동구동) ----
    def step_physics(self, wL, wR, wheel_r, wheel_base, dt):
        v = (wR + wL) * wheel_r * 0.5
        w = (wR - wL) * wheel_r / wheel_base
        self.robot.v, self.robot.w = v, w

        th = self.robot.theta
        if abs(w) < 1e-6:
            self.robot.x += v * math.cos(th) * dt
            self.robot.y += v * math.sin(th) * dt
        else:
            r = v / w
            dth = w * dt
            self.robot.x += r * (math.sin(th + dth) - math.sin(th))
            self.robot.y -= r * (math.cos(th + dth) - math.cos(th))
            self.robot.theta = (th + dth + math.pi) % (2 * math.pi) - math.pi

        self._update_collisions(wheel_base)
        self._update_regions(wheel_base, dt)

    def _robot_radius(self):
        return getattr(self, "_rr", 0.13)

    def set_robot_radius(self, r):
        self._rr = r

    def _update_collisions(self, wheel_base):
        rr = self._robot_radius()
        hit = False
        for o in self.obstacles:
            if o.distance_to_point(self.robot.x, self.robot.y) < rr:
                hit = True
                break
        if not hit and self.walls_on:
            x, y = self.robot.x, self.robot.y
            if x - rr < 0 or x + rr > ARENA_M or y - rr < 0 or y + rr > ARENA_M:
                hit = True
        # 새로 충돌이 시작될 때만 1회 카운트(디바운스)
        if hit and not self._colliding:
            self.collisions += 1
        self._colliding = hit

    def _update_regions(self, wheel_base, dt):
        if self.finished:
            return
        color = self.current_target_color()
        if color is None:
            self.finished = True
            return
        left, right = self.robot.wheel_contacts(wheel_base)
        inside = any(
            p.color == color and p.contains(*left) and p.contains(*right)
            for p in self.patches
        )
        stopped = abs(self.robot.v) < 0.02
        if inside and stopped:
            self.dwell += dt
            if self.dwell >= 1.0:
                self.cleared[self.target_idx] = True
                self.target_idx += 1
                self.dwell = 0.0
                if self.target_idx >= len(ORDER):
                    self.finished = True
        else:
            self.dwell = 0.0
