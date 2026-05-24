# -*- coding: utf-8 -*-
"""
실제 project3.py / project3.ino 코드를 시뮬레이터로 불러오는 다리(bridge).

  * project3.py  : serial 모듈을 스텁으로 주입한 뒤 importlib로 로드.
                   → choose_cmd / lidar_points_to_xy / predict_xy / 모든 상수가
                     '실제 코드' 그대로 시뮬레이터에서 호출된다(sim-to-real 핵심).
  * project3.ino : const / #define 상수를 정규식으로 파싱(휠 반지름, 폭, 한계,
                   CMD_TIMEOUT 등). .ino를 수정하면 이 값들이 자동 반영된다.
  * hot-reload   : 두 파일의 mtime을 감시해 변경 시 다시 로드한다.
"""

import importlib.util
import math
import os
import re
import sys
import types

import numpy as np

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT3_DIR = os.path.dirname(_THIS_DIR)            # .../project3
PY_PATH = os.path.join(PROJECT3_DIR, "project3.py")
INO_PATH = os.path.join(PROJECT3_DIR, "project3.ino")


# ---------------------------------------------------------------- 스텁 주입
def _install_stubs():
    """project3.py가 하드웨어 없이 import되도록 serial/picamera2를 가짜로 채운다.
       (시뮬레이터는 실제 시리얼/카메라를 절대 호출하지 않는다.)"""
    if not getattr(sys.modules.get("serial"), "_sim_stub", False):
        m = types.ModuleType("serial")
        m._sim_stub = True

        class _Serial:
            def __init__(self, *a, **k):
                pass

            def read(self, *a, **k):
                return b""

            def readline(self, *a, **k):
                return b""

            def write(self, *a, **k):
                return 0

            def reset_input_buffer(self):
                pass

            def close(self):
                pass

            @property
            def in_waiting(self):
                return 0

        m.Serial = _Serial
        m.SerialException = Exception
        sys.modules["serial"] = m

    if "picamera2" not in sys.modules:
        pm = types.ModuleType("picamera2")
        pm.Picamera2 = object
        sys.modules["picamera2"] = pm


# ---------------------------------------------------------------- project3.py
def load_p3():
    """project3.py를 새 모듈 객체로 로드해 반환."""
    _install_stubs()
    spec = importlib.util.spec_from_file_location("project3_live", PY_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def recompute_derived(p3):
    """슬라이더로 기본 파라미터를 바꾼 뒤, 그 값에서 파생되는 상수들을 다시 계산.
       (파일 수정 후 reload 시에는 모듈 최상단이 재실행되므로 호출 불필요.)"""
    p3.W_SET = np.linspace(-p3.MAX_W, p3.MAX_W, 11)
    if hasattr(p3, "V_SET_RATIOS") and hasattr(p3, "V_SET_MIN"):
        p3.V_SET = np.maximum(p3.V_SET_MIN, p3.CRUISE_V * p3.V_SET_RATIOS)
    p3.STEPS = int(p3.HORIZON_T / p3.DT)
    p3._TS = (np.arange(p3.STEPS) + 1) * p3.DT
    p3.HALF_HFOV_RAD = math.radians(p3.HFOV_DEG) * 0.5
    p3.CAM_AREA = float(p3.CAM_W * p3.CAM_H)
    if hasattr(p3, "PRACTICE10_FX_640"):
        p3.CAMERA_FX_PX = p3.PRACTICE10_FX_640 * (p3.CAM_W / 640.0)
        p3.CAMERA_FY_PX = p3.PRACTICE10_FY_480 * (p3.CAM_H / 480.0)
        p3.CAMERA_CX_PX = p3.PRACTICE10_CX_640 * (p3.CAM_W / 640.0)
        p3.CAMERA_CY_PX = p3.PRACTICE10_CY_480 * (p3.CAM_H / 480.0)
    if hasattr(p3, "BLIND_CREEP_DIST_M"):
        p3.BLIND_CREEP_S = p3.BLIND_CREEP_DIST_M / max(1e-6, p3.BLIND_CREEP_V)
    if hasattr(p3, "SEARCH_SWEEP_ANGLE_DEG"):
        p3.SEARCH_SWEEP_ANGLE_RAD = math.radians(p3.SEARCH_SWEEP_ANGLE_DEG)
        p3.SEARCH_BEARING = p3.SEARCH_SWEEP_ANGLE_RAD
    if hasattr(p3, "ODOM_PPR"):
        p3.ODOM_COUNT_PER_RAD = p3.ODOM_PPR / (2.0 * math.pi)


# ---------------------------------------------------------------- project3.ino
_DEFINE_RE = re.compile(r"#define\s+(\w+)\s+([^\n/]+)")
_CONST_RE = re.compile(
    r"const\s+(?:unsigned\s+long|float|double|int|long|byte|bool)\s+(\w+)\s*=\s*([^;]+);"
)


def _strip_c_number_suffix(expr):
    # 0.034f -> 0.034 (소문자 f 접미사만; PI_F 같은 식별자는 보존)
    return re.sub(r"([0-9.])[fF](?![A-Za-z0-9_])", r"\1", expr)


def _eval_c_value(expr, ns):
    expr = _strip_c_number_suffix(expr).strip()
    safe = {"__builtins__": {}}
    safe.update(ns)
    safe.update({"true": True, "false": False, "PI": math.pi})
    try:
        return eval(expr, safe)  # noqa: S307 - 신뢰된 소스(.ino) 상수식만
    except Exception:
        try:
            return float(expr)
        except Exception:
            return None


def parse_ino(path=INO_PATH):
    """.ino의 #define + const 상수를 {이름: 값} 으로 반환."""
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        txt = f.read()

    ns = {}
    for name, raw in _DEFINE_RE.findall(txt):
        val = _eval_c_value(raw, ns)
        if val is not None:
            ns[name] = val
    for name, raw in _CONST_RE.findall(txt):
        val = _eval_c_value(raw, ns)
        if val is not None:
            ns[name] = val
    return ns


# ---------------------------------------------------------------- hot-reload
class CodeBridge:
    """project3.py / project3.ino를 로드하고 변경을 감시한다."""

    def __init__(self):
        self.p3 = load_p3()
        self.ino = parse_ino()
        self._py_mtime = os.path.getmtime(PY_PATH)
        self._ino_mtime = os.path.getmtime(INO_PATH)

    def check_reload(self):
        """변경된 파일이 있으면 다시 로드. (reloaded_py, reloaded_ino) 반환."""
        reloaded_py = reloaded_ino = False
        try:
            m = os.path.getmtime(PY_PATH)
            if m != self._py_mtime:
                self.p3 = load_p3()
                self._py_mtime = m
                reloaded_py = True
        except OSError:
            pass
        try:
            m = os.path.getmtime(INO_PATH)
            if m != self._ino_mtime:
                self.ino = parse_ino()
                self._ino_mtime = m
                reloaded_ino = True
        except OSError:
            pass
        return reloaded_py, reloaded_ino


if __name__ == "__main__":
    b = CodeBridge()
    print("[p3] CRUISE_V =", b.p3.CRUISE_V, " MAX_W =", b.p3.MAX_W,
          " COLLISION_DIST =", b.p3.COLLISION_DIST)
    print("[ino] WHEEL_R =", b.ino.get("WHEEL_R"),
          " WHEEL_BASE =", b.ino.get("WHEEL_BASE"),
          " CMD_TIMEOUT_MS =", b.ino.get("CMD_TIMEOUT_MS"),
          " WHEEL_SPEED_MAX =", b.ino.get("WHEEL_SPEED_MAX"))
