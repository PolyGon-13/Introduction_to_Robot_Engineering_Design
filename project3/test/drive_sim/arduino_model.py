# -*- coding: utf-8 -*-
"""
project3.ino 를 시뮬레이션하는 아두이노 모델.

충실히 재현(.ino 상수 그대로 사용, 파일 수정 시 자동 반영):
  * 시리얼 명령 파싱  "V<v>,<w>\\n" / "S\\n"   (processCommand 동일)
  * (v,w) → 좌우 바퀴 목표 각속도            (resolveWheelTargets 동일)
  * WHEEL_SPEED_MAX 클램프, V_cmd==0&&W_cmd==0 즉시 정지
  * CMD_TIMEOUT_MS: 명령이 끊기면 정지        (안전 동작)

추상화한 부분(README 참고):
  * 모터+PID 폐루프 과도응답을 1차 지연(τ) + 가속 한계로 모델링.
    실제 PID 게인까지 재현하려면 검증된 모터 전기 파라미터가 필요하므로,
    여기서는 폐루프 상승시간을 τ 하나로 근사하고 sim 파라미터로 조정 가능하게 둔다.
"""


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class ArduinoSim:
    def __init__(self, ino):
        self.ino = ino                 # .ino 상수 dict (code_bridge.parse_ino)
        self.V_cmd = 0.0               # 목표 선속도 (m/s)
        self.W_cmd = 0.0               # 목표 각속도 (rad/s)
        self.wL = 0.0                  # 실제 좌바퀴 각속도 (rad/s)
        self.wR = 0.0                  # 실제 우바퀴 각속도 (rad/s)
        self.now = 0.0                 # 내부 시계 (s)
        self.last_cmd_t = 0.0
        self._rx = ""

        # --- sim 파라미터 (GUI에서 조정) ---
        self.tau = 0.10                # 휠 폐루프 응답 시간상수 (s)
        self.wheel_accel_max = 50.0    # 휠 각가속 한계 (rad/s^2)

    # 실제 project3.send_vw/stop 이 호출하는 serial.write 와 동일 인터페이스
    def write(self, data):
        s = data.decode() if isinstance(data, (bytes, bytearray)) else str(data)
        self._rx += s
        while "\n" in self._rx:
            line, self._rx = self._rx.split("\n", 1)
            self._process(line)
        return len(data)

    def _process(self, s):
        s = s.strip()
        if not s:
            return
        c0 = s[0]
        if c0 in "Ss":
            self.V_cmd = 0.0
            self.W_cmd = 0.0
            self.last_cmd_t = self.now
        elif c0 in "Vv":
            comma = s.find(",")
            if comma <= 1 or comma >= len(s) - 1:
                return
            try:
                self.V_cmd = float(s[1:comma])
                self.W_cmd = float(s[comma + 1:])
                self.last_cmd_t = self.now
            except ValueError:
                pass

    def _resolve_targets(self):
        R = self.ino["WHEEL_R"]
        WB = self.ino["WHEEL_BASE"]
        cap = self.ino["WHEEL_SPEED_MAX"]
        wL = (self.V_cmd - WB * self.W_cmd * 0.5) / R
        wR = (self.V_cmd + WB * self.W_cmd * 0.5) / R
        return _clamp(wL, -cap, cap), _clamp(wR, -cap, cap)

    def _approach(self, cur, tgt, dt):
        # 1차 지연으로 목표에 접근 + 가속 한계
        step = (tgt - cur) * min(1.0, dt / max(self.tau, 1e-3))
        amax = self.wheel_accel_max * dt
        step = _clamp(step, -amax, amax)
        return cur + step

    def update(self, dt):
        self.now += dt
        # CMD_TIMEOUT: 명령 끊기면 정지 (.ino 와 동일)
        if (self.now - self.last_cmd_t) * 1000.0 > self.ino["CMD_TIMEOUT_MS"]:
            self.V_cmd = 0.0
            self.W_cmd = 0.0

        if self.V_cmd == 0.0 and self.W_cmd == 0.0:
            tL = tR = 0.0
        else:
            tL, tR = self._resolve_targets()
        self.wL = self._approach(self.wL, tL, dt)
        self.wR = self._approach(self.wR, tR, dt)

    # --- 텔레메트리: 실제 (v,w) ---
    def actual_vw(self):
        R = self.ino["WHEEL_R"]
        WB = self.ino["WHEEL_BASE"]
        v = (self.wR + self.wL) * R * 0.5
        w = (self.wR - self.wL) * R / WB
        return v, w
