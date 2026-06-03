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
def camera_rect_half_width(p3, cam_max=None, cam_width_m=None):
    """top-down 카메라 바닥 직사각형의 좌우 반폭."""
    if cam_width_m is not None:
        return max(0.01, cam_width_m * 0.5)
    return max(0.01, cam_max * math.tan(p3.HALF_HFOV_RAD))


def camera_forward_range(cam_min=0.0, cam_max=2.5,
                         cam_rear_blind_m=None, cam_depth_m=None,
                         robot_radius_m=None, robot_body_length_m=None,
                         camera_forward_offset_m=0.0):
    """카메라 기준 직사각형의 전방 시작/끝 x 좌표.

    cam_rear_blind_m 은 로봇의 가장 뒤쪽 점에서부터 보이지 않는 전방 거리다.
    로봇이 직사각형이면 rear point=-body_length/2, 원형이면 -radius 로 본다.
    """
    if (cam_rear_blind_m is None or cam_depth_m is None or
            (robot_radius_m is None and robot_body_length_m is None)):
        return cam_min, cam_max
    rear_x_robot = (
        -robot_body_length_m * 0.5
        if robot_body_length_m is not None else -robot_radius_m
    )
    near_x_robot = rear_x_robot + cam_rear_blind_m
    near_x_camera = near_x_robot - camera_forward_offset_m
    return near_x_camera, near_x_camera + cam_depth_m


def camera_world_pose(robot, camera_forward_offset_m=0.0, camera_left_offset_m=0.0):
    """로봇 로컬 오프셋(x=전방, y=좌측)을 월드 카메라 위치로 변환."""
    cs = math.cos(robot.theta)
    sn = math.sin(robot.theta)
    return (
        robot.x + camera_forward_offset_m * cs - camera_left_offset_m * sn,
        robot.y + camera_forward_offset_m * sn + camera_left_offset_m * cs,
    )


def _polygon_area(poly):
    if len(poly) < 3:
        return 0.0
    acc = 0.0
    for i, p0 in enumerate(poly):
        p1 = poly[(i + 1) % len(poly)]
        acc += p0[0] * p1[1] - p1[0] * p0[1]
    return 0.5 * acc


def _polygon_centroid(poly):
    area = _polygon_area(poly)
    if abs(area) < 1e-12:
        if not poly:
            return 0.0, 0.0
        return (
            sum(p[0] for p in poly) / len(poly),
            sum(p[1] for p in poly) / len(poly),
        )
    cx = cy = 0.0
    for i, p0 in enumerate(poly):
        p1 = poly[(i + 1) % len(poly)]
        cross = p0[0] * p1[1] - p1[0] * p0[1]
        cx += (p0[0] + p1[0]) * cross
        cy += (p0[1] + p1[1]) * cross
    scale = 1.0 / (6.0 * area)
    return cx * scale, cy * scale


def _clip_axis(poly, axis, value, keep_greater):
    if not poly:
        return []

    def inside(p):
        return p[axis] >= value if keep_greater else p[axis] <= value

    def intersect(a, b):
        da = a[axis] - value
        db = b[axis] - value
        denom = da - db
        t = 0.0 if abs(denom) < 1e-12 else da / denom
        return (
            a[0] + t * (b[0] - a[0]),
            a[1] + t * (b[1] - a[1]),
        )

    out = []
    prev = poly[-1]
    prev_in = inside(prev)
    for cur in poly:
        cur_in = inside(cur)
        if cur_in:
            if not prev_in:
                out.append(intersect(prev, cur))
            out.append(cur)
        elif prev_in:
            out.append(intersect(prev, cur))
        prev, prev_in = cur, cur_in
    return out


def _clip_to_camera_rect(poly, near_x, far_x, half_width):
    clipped = _clip_axis(poly, 0, near_x, True)
    clipped = _clip_axis(clipped, 0, far_x, False)
    clipped = _clip_axis(clipped, 1, -half_width, True)
    clipped = _clip_axis(clipped, 1, half_width, False)
    return clipped


def patch_camera_overlap(world, pch, near_x, far_x, half_width,
                         camera_forward_offset_m=0.0, camera_left_offset_m=0.0):
    """카메라 직사각형과 색종이 사각형의 겹침을 카메라 프레임에서 계산."""
    rb = world.robot
    cam_x, cam_y = camera_world_pose(rb, camera_forward_offset_m, camera_left_offset_m)
    cs = math.cos(rb.theta)
    sn = math.sin(rb.theta)
    h = pch.size * 0.5
    poly = []
    for wx, wy in (
        (pch.x - h, pch.y - h),
        (pch.x + h, pch.y - h),
        (pch.x + h, pch.y + h),
        (pch.x - h, pch.y + h),
    ):
        rx = wx - cam_x
        ry = wy - cam_y
        poly.append((rx * cs + ry * sn, -rx * sn + ry * cs))

    clipped = _clip_to_camera_rect(poly, near_x, far_x, half_width)
    overlap_area = abs(_polygon_area(clipped))
    if overlap_area <= 1e-10:
        return {
            "visible": False,
            "area_m2": 0.0,
            "ratio": 0.0,
            "centroid": (0.0, 0.0),
            "polygon": [],
        }
    cx, cy = _polygon_centroid(clipped)
    return {
        "visible": True,
        "area_m2": overlap_area,
        "ratio": overlap_area / max(1e-12, pch.size * pch.size),
        "centroid": (cx, cy),
        "polygon": clipped,
    }


def segment_enter_fraction_obstacle(x0, y0, x1, y1, obstacle):
    """선분이 회전 직사각형 발자국에 들어가는 0..1 비율. 미교차면 None."""
    ox, oy = obstacle.world_to_local(x0, y0)
    ex, ey = obstacle.world_to_local(x1, y1)
    dx = ex - ox
    dy = ey - oy
    t0, t1 = 0.0, 1.0

    for origin, delta, lo, hi in (
        (ox, dx, -obstacle.half_w, obstacle.half_w),
        (oy, dy, -obstacle.half_h, obstacle.half_h),
    ):
        if abs(delta) < 1e-9:
            if origin < lo or origin > hi:
                return None
            continue
        ta = (lo - origin) / delta
        tb = (hi - origin) / delta
        if ta > tb:
            ta, tb = tb, ta
        t0 = max(t0, ta)
        t1 = min(t1, tb)
        if t0 > t1:
            return None
    return max(0.0, t0)


def camera_blocker(world, camera_x, camera_y, target_x, target_y, camera_height_m):
    """카메라-바닥 목표 시야선을 가리는 장애물 반환. 없으면 None."""
    best = None
    for obstacle in world.obstacles:
        frac = segment_enter_fraction_obstacle(camera_x, camera_y, target_x, target_y, obstacle)
        if frac is None or frac > 1.0:
            continue
        line_z = camera_height_m * (1.0 - frac)
        if obstacle.z_h >= line_z:
            if best is None or frac < best[1]:
                best = (obstacle, frac, line_z)
    return best


def simulate_camera(world, p3, target_color, cam_max=2.5, cam_far=1.5,
                    bearing_noise_deg=0.0, cam_min=0.0, camera_height_m=0.18,
                    cam_width_m=None, cam_depth_m=None,
                    cam_rear_blind_m=None, robot_radius_m=None,
                    robot_body_length_m=None, camera_forward_offset_m=0.0,
                    camera_left_offset_m=0.0):
    """기하 기반 인식. 실제 상수(BEARING_SIGN, CAM_W/H)를 사용.
       로봇 상단 카메라가 바닥을 내려다본다고 보고 직사각형 발자국만 본다.
       패치 중심이 아니라 카메라 footprint와 겹친 영역의 중심을 추적한다.
       반환: (visible, bearing_rad, area_ratio, cy_norm, n_visible, center_x_robot_m, center_y_robot_m)."""
    rb = world.robot
    half_fov = p3.HALF_HFOV_RAD
    focal_px = (p3.CAM_W * 0.5) / max(1e-6, math.tan(half_fov))
    near_x, far_x = camera_forward_range(
        cam_min, cam_max, cam_rear_blind_m, cam_depth_m, robot_radius_m,
        robot_body_length_m, camera_forward_offset_m)
    half_width = camera_rect_half_width(p3, far_x, cam_width_m)
    cam_x, cam_y = camera_world_pose(rb, camera_forward_offset_m, camera_left_offset_m)

    best = None
    best_area_px = 0.0
    n_visible = 0

    cs = math.cos(rb.theta)
    sn = math.sin(rb.theta)
    for pch in world.patches:
        if pch.color != target_color:
            continue
        overlap = patch_camera_overlap(
            world, pch, near_x, far_x, half_width,
            camera_forward_offset_m, camera_left_offset_m)
        if not overlap["visible"]:
            continue
        x_r, y_r = overlap["centroid"]
        overlap_world_x = cam_x + x_r * cs - y_r * sn
        overlap_world_y = cam_y + x_r * sn + y_r * cs
        if camera_blocker(world, cam_x, cam_y, overlap_world_x, overlap_world_y, camera_height_m) is not None:
            continue
        dx = pch.x - rb.x
        dy = pch.y - rb.y
        center_x_robot = dx * cs + dy * sn
        center_y_robot = -dx * sn + dy * cs
        image_err = -max(-1.0, min(1.0, y_r / half_width))
        bearing = p3.BEARING_SIGN * image_err * half_fov
        projection_distance = x_r if x_r > 0.0 else math.hypot(x_r, y_r)
        area_px = (
            focal_px * focal_px * overlap["area_m2"] /
            max(1e-6, projection_distance * projection_distance)
        )
        n_visible += 1
        if area_px > best_area_px:
            best_area_px = area_px
            best = (bearing, x_r, center_x_robot, center_y_robot)

    if best is None:
        return (False, 0.0, 0.0, 0.0, 0, None, None)

    bearing, forward_x, center_x_robot, center_y_robot = best
    if bearing_noise_deg > 0.0:
        bearing += math.radians(np.random.normal(0.0, bearing_noise_deg))
    area_ratio = best_area_px / p3.CAM_AREA
    cy_span = max(1e-6, far_x - near_x)
    cy_norm = max(0.0, min(1.0, 1.0 - (forward_x - near_x) / cy_span))
    return (
        True,
        float(bearing),
        float(area_ratio),
        float(cy_norm),
        n_visible,
        float(center_x_robot),
        float(center_y_robot),
    )
