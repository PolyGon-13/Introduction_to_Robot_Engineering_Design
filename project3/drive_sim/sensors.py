# -*- coding: utf-8 -*-
"""
합성 센서. 핵심: 라이다는 '원시 튜플'을 만들어 실제 project3.lidar_points_to_xy
가 변환하도록 하고(캘리브/필터/전방필터/서브샘플 전부 실제 코드 경유),
카메라는 실제 상수(HFOV, BEARING_SIGN)로 (visible, β, area, cy)를 만든다.
"""

import math
import numpy as np

from world import ARENA_M


# ===================== 라이다 =====================
def simulate_lidar_raw(world, p3, n_rays=360, sensor_max=6.0, noise_m=0.0):
    """장애물(회전 직사각형)+벽에 레이캐스팅 → 실제 코드가 기대하는 (angles_deg, dists_mm, q)."""
    rb = world.robot
    phi = np.linspace(-math.pi, math.pi, n_rays, endpoint=False)   # 로봇기준 방위
    A = rb.theta + phi
    dx = np.cos(A)
    dy = np.sin(A)

    dist = np.full(n_rays, np.inf)

    # 회전 직사각형 장애물: 레이를 장애물 로컬 좌표로 돌린 뒤 slab 교차.
    for o in world.obstacles:
        ox, oy = o.world_to_local(rb.x, rb.y)
        c = math.cos(o.theta)
        s = math.sin(o.theta)
        ldx = c * dx + s * dy
        ldy = -s * dx + c * dy
        hw, hh = o.half_w, o.half_h
        inside_x = -hw <= ox <= hw
        inside_y = -hh <= oy <= hh
        parallel_x = np.abs(ldx) < 1e-9
        parallel_y = np.abs(ldy) < 1e-9

        with np.errstate(divide="ignore", invalid="ignore"):
            tx1 = (-hw - ox) / ldx
            tx2 = (hw - ox) / ldx
            ty1 = (-hh - oy) / ldy
            ty2 = (hh - oy) / ldy

        tx_min = np.minimum(tx1, tx2)
        tx_max = np.maximum(tx1, tx2)
        ty_min = np.minimum(ty1, ty2)
        ty_max = np.maximum(ty1, ty2)

        tx_min = np.where(parallel_x & inside_x, -np.inf, tx_min)
        tx_max = np.where(parallel_x & inside_x, np.inf, tx_max)
        tx_min = np.where(parallel_x & (not inside_x), np.inf, tx_min)
        tx_max = np.where(parallel_x & (not inside_x), -np.inf, tx_max)
        ty_min = np.where(parallel_y & inside_y, -np.inf, ty_min)
        ty_max = np.where(parallel_y & inside_y, np.inf, ty_max)
        ty_min = np.where(parallel_y & (not inside_y), np.inf, ty_min)
        ty_max = np.where(parallel_y & (not inside_y), -np.inf, ty_max)

        t_enter = np.maximum(tx_min, ty_min)
        t_exit = np.minimum(tx_max, ty_max)
        t = np.where(t_enter > 1e-4, t_enter, t_exit)
        valid = (t_exit >= np.maximum(t_enter, 1e-4)) & (t > 1e-4)
        dist = np.where(valid, np.minimum(dist, t), dist)

    # 아레나 벽 (내부에서 박스 경계까지)
    if world.walls_on:
        with np.errstate(divide="ignore", invalid="ignore"):
            tx = np.where(dx > 0, (ARENA_M - rb.x) / dx,
                          np.where(dx < 0, (0.0 - rb.x) / dx, np.inf))
            ty = np.where(dy > 0, (ARENA_M - rb.y) / dy,
                          np.where(dy < 0, (0.0 - rb.y) / dy, np.inf))
        wall_t = np.minimum(tx, ty)
        dist = np.minimum(dist, wall_t)

    if noise_m > 0.0:
        dist = dist + np.random.normal(0.0, noise_m, size=n_rays)

    keep = np.isfinite(dist) & (dist > 0) & (dist < sensor_max)
    if not keep.any():
        empty = np.empty(0, dtype=np.float32)
        return empty, empty, empty

    phi_k = phi[keep]
    dist_k = dist[keep]

    # project3 캘리브의 역변환: raw_deg → (코드가 적용 후) phi 가 되도록
    #   코드: phi_deg = LIDAR_ANGLE_SIGN * normalize(raw + ANGLE_OFFSET_DEG)
    raw_deg = p3.LIDAR_ANGLE_SIGN * np.degrees(phi_k) - p3.ANGLE_OFFSET_DEG
    dist_mm = dist_k * 1000.0 - p3.DIST_OFFSET_MM
    q = np.full(phi_k.shape, 15.0, dtype=np.float32)
    return (raw_deg.astype(np.float32), dist_mm.astype(np.float32), q)


# ===================== 카메라 =====================
def simulate_camera(world, p3, target_color, cam_max=2.5, cam_far=1.5,
                    bearing_noise_deg=0.0):
    """기하 기반 인식. 실제 상수(HALF_HFOV_RAD, BEARING_SIGN, CAM_W/H, MIN_AREA) 사용.
       반환: (visible, bearing_rad, area_ratio, cy_norm, n_visible)."""
    rb = world.robot
    half_fov = p3.HALF_HFOV_RAD
    focal_px = (p3.CAM_W * 0.5) / max(1e-6, math.tan(half_fov))

    best = None
    best_area_px = 0.0
    n_visible = 0

    cs = math.cos(rb.theta)
    sn = math.sin(rb.theta)
    for pch in world.patches:
        if pch.color != target_color:
            continue
        rx = pch.x - rb.x
        ry = pch.y - rb.y
        x_r = rx * cs + ry * sn       # 전방
        y_r = -rx * sn + ry * cs      # 좌측(+)
        if x_r <= 0.0:
            continue
        d = math.hypot(x_r, y_r)
        if d > cam_max:
            continue
        bearing_true = math.atan2(y_r, x_r)
        if abs(bearing_true) > half_fov:
            continue
        side_px = focal_px * pch.size / max(1e-6, d)
        area_px = side_px * side_px
        if area_px < p3.MIN_AREA:        # 너무 멀어 작게 보이면 미검출(실제 필터)
            continue
        n_visible += 1
        if area_px > best_area_px:
            best_area_px = area_px
            best = (bearing_true, d)

    if best is None:
        return (False, 0.0, 0.0, 0.0, 0)

    bearing_true, d = best
    # 직립 카메라 가정: 로봇 좌측(+bearing) → 이미지 좌측(-err)
    err = -bearing_true / max(1e-6, half_fov)
    err = max(-1.0, min(1.0, err))
    # 실제 _detect 와 동일한 부호식 → BEARING_SIGN 을 그대로 검증
    bearing = p3.BEARING_SIGN * err * half_fov
    if bearing_noise_deg > 0.0:
        bearing += math.radians(np.random.normal(0.0, bearing_noise_deg))
    area_ratio = best_area_px / p3.CAM_AREA
    cy_norm = max(0.0, min(1.0, 1.0 - d / max(1e-6, cam_far)))
    return (True, float(bearing), float(area_ratio), float(cy_norm), n_visible)
