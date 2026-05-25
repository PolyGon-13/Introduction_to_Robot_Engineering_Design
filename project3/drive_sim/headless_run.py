#!/usr/bin/env python3
"""
헤드리스 배치 시뮬레이터 러너.
project3.py 로직을 layout.json 기준으로 N회 반복 실행하고 결과를 출력한다.

  사용법:  python3 headless_run.py [N]
           N 생략 시 3회 실행
"""
import os
import sys
import math
import time

# pygame dummy 드라이버는 import 전에 설정해야 한다.
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

# drive_sim 디렉토리를 작업 디렉토리 및 import 경로로 설정
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
os.chdir(_SCRIPT_DIR)
sys.path.insert(0, _SCRIPT_DIR)

import pygame
pygame.init()
pygame.display.set_mode((1, 1))

import sim as _sim_module
from sim import Simulator

LOG_INTERVAL_S = 5.0    # 콘솔 로그 주기 (시뮬 시간 기준)
STUCK_DIST_M = 0.05     # 5초 동안 이 거리 미만이면 'stuck'
MAX_SIM_TIME = 180.0


def run_once(trial: int) -> dict:
    _sim_module.configure_layout(_sim_module.WIN_W, _sim_module.WIN_H)
    sim = Simulator()
    sim.load_layout()
    sim.start()

    SIM_DT = sim.SIM_DT
    ino = sim.bridge.ino

    last_log_t = -LOG_INTERVAL_S      # 0초에 첫 로그 찍기
    last_log_pos = (sim.world.robot.x, sim.world.robot.y)
    stuck_events: list[str] = []
    phase_log: list[tuple] = []       # (t, phase, x, y)

    wall_start = time.perf_counter()

    while sim.sim_time < MAX_SIM_TIME and not sim.world.finished:
        sim.brain_acc += SIM_DT
        if sim.brain_acc >= sim.p3.LOOP_DT:
            sim.brain_acc -= sim.p3.LOOP_DT
            sim.brain_tick(commit=True)

        sim.arduino.update(SIM_DT)
        sim.world.step_physics(
            sim.arduino.wL, sim.arduino.wR,
            ino["WHEEL_R"], ino["WHEEL_BASE"], SIM_DT
        )
        sim.sim_time += SIM_DT

        if sim.sim_time - last_log_t >= LOG_INTERVAL_S:
            rb = sim.world.robot
            phase = getattr(sim.ctrl, "phase", "?")
            cleared_str = "".join(
                (c[0] if sim.world.cleared[i] else c[0].lower())
                for i, c in enumerate(["RED", "YELLOW", "BLUE"])
            )
            tgt = sim.ctrl.target_color if hasattr(sim.ctrl, "target_color") else "?"
            rescan = getattr(sim.ctrl, "rescan_dist_m", None)
            rescan_s = f"  rescan={rescan:.2f}m" if rescan is not None else ""
            print(
                f"  t={sim.sim_time:6.1f}s "
                f"pos=({rb.x:+.2f},{rb.y:+.2f}) "
                f"θ={math.degrees(rb.theta):+.0f}° "
                f"phase={phase:<16} "
                f"target={tgt:<8} "
                f"cleared={cleared_str} "
                f"col={sim.world.collisions}"
                + rescan_s
            )
            dist_5s = math.hypot(rb.x - last_log_pos[0], rb.y - last_log_pos[1])
            if dist_5s < STUCK_DIST_M:
                msg = (
                    f"t={sim.sim_time:.0f}s: STUCK "
                    f"({dist_5s:.3f}m/{LOG_INTERVAL_S:.0f}s) "
                    f"phase={phase} pos=({rb.x:+.2f},{rb.y:+.2f})"
                )
                stuck_events.append(msg)
                print(f"    *** {msg}")
            phase_log.append((sim.sim_time, phase, rb.x, rb.y))
            last_log_pos = (rb.x, rb.y)
            last_log_t = sim.sim_time

    wall_elapsed = time.perf_counter() - wall_start
    return {
        "trial": trial,
        "sim_time": sim.sim_time,
        "finished": sim.world.finished,
        "cleared": list(sim.world.cleared),
        "collisions": sim.world.collisions,
        "stuck_events": stuck_events,
        "final_phase": getattr(sim.ctrl, "phase", "?"),
        "wall_elapsed_s": wall_elapsed,
        "phase_log": phase_log,
    }


def main():
    n_trials = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    print(f"=== headless_run: {n_trials}회 실행 ===\n")

    results = []
    for i in range(1, n_trials + 1):
        print(f"[Trial {i}/{n_trials}]")
        r = run_once(i)
        results.append(r)
        status = "FINISHED" if r["finished"] else f"TIMEOUT t={r['sim_time']:.1f}s"
        cleared_labels = ["RED", "YELLOW", "BLUE"]
        cleared_str = ", ".join(
            l for l, ok in zip(cleared_labels, r["cleared"]) if ok
        ) or "(없음)"
        print(
            f"  → {status} | cleared: {cleared_str} | "
            f"col={r['collisions']} | wall={r['wall_elapsed_s']:.1f}s"
        )
        if r["stuck_events"]:
            print(f"  stuck 이벤트 {len(r['stuck_events'])}건:")
            for e in r["stuck_events"]:
                print(f"    - {e}")
        print()

    # 요약
    print("=" * 60)
    print("요약")
    print("=" * 60)
    n_fin = sum(1 for r in results if r["finished"])
    n_col = sum(1 for r in results if r["collisions"] > 0)
    print(f"완주: {n_fin}/{n_trials}  충돌 있음: {n_col}/{n_trials}")
    for r in results:
        cleared_str = "".join(
            (c[0] if r["cleared"][i] else c[0].lower())
            for i, c in enumerate(["RED", "YELLOW", "BLUE"])
        )
        print(
            f"  Trial {r['trial']}: cleared={cleared_str}  "
            f"t={r['sim_time']:.1f}s  col={r['collisions']}  "
            f"final_phase={r['final_phase']}"
        )

    # 공통 stuck 패턴 분석
    all_stuck = [e for r in results for e in r["stuck_events"]]
    if all_stuck:
        print(f"\n전체 stuck 이벤트 ({len(all_stuck)}건):")
        for e in all_stuck:
            print(f"  {e}")


if __name__ == "__main__":
    main()
