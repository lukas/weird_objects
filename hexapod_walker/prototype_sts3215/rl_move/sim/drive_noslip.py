"""Interactive drive-around for the no-slip step-then-shift gait.

Same battle-tested render path as drive_policy: the env's offscreen
renderer + cv2.imshow, plain python (NOT mjpython — its viewer
segfaults intermittently on macOS):

    cd hexapod_walker/prototype_sts3215
    uv run python -m rl_move.sim.drive_noslip
    # or: sim_viewer/sim_noslip.sh

Controls (the "noslip drive" window must have focus; also drawn in it):
    I / Up      more forward speed      K / Down    less / backward
    J / Left    strafe left             L / Right   strafe right
    U / O       turn left / right (the gait supports omega directly)
    0 / Space   full stop               R           reset episode
    1 / 2 / 3   gait period 3.2 / 2.4 / 2.0 s (rebuilds the gait; the
                slower the period the cleaner the no-slip tracking)
    4 / 5 / 6   alpha 0 / 0.5 / 1.0 — body-motion overlap (rebuilds the
                gait): 0 = step-then-shift, 1 = continuous body drift;
                feet stay world-pinned at every alpha
    7           CLAMP-FIT preset (NoSlipGait.CLAMP_FIT_KW: period 6 s,
                swing-heavy, alpha 1) — the 08-12 sweep's cleanest
                timing under the fitted ~31 deg/s servo clamp
    Q / Esc     quit

Runs in the verify_noslip env (goal env pinned to hold, hardware-like
write profile 1500/80 with the servo cruise unclamped) — the setting
where the gait was verified to have zero foot scrub.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from rl_move.robot_state import DEG2RAD  # noqa: E402

_STEP_V = 0.005      # m/s per tap
_STEP_W = 0.05       # rad/s per tap
_HOLD_S = 1.0        # settle at plant after every reset
_PERIODS = {ord("1"): 3.2, ord("2"): 2.4, ord("3"): 2.0}
_ALPHAS = {ord("4"): 0.0, ord("5"): 0.5, ord("6"): 1.0}

# cv2.waitKeyEx codes: macOS arrow keys.
_UP, _DOWN, _LEFT, _RIGHT = 63232, 63233, 63234, 63235


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--mu", type=float, default=0.0,
                    help="foot-ground slide friction (0 = XML default 2.0)")
    ap.add_argument("--realtime", type=float, default=1.0,
                    help="1.0 = wall clock, 2.0 = 2x, 0 = uncapped")
    args = ap.parse_args()

    import cv2

    from sim_gait_compat import NoSlipGait
    from rl_move.sim.joint_task import q_rad_to_action
    from rl_move.sim.verify_noslip import (_make_env, PLANT_HIP_DEG,
                                           PLANT_KNEE_DEG)

    env = _make_env(args.mu, "", 0, episode_s=3600.0, render=True,
                    write_speed=1500, write_acc=80)
    plant_rad = np.array([0.0, PLANT_HIP_DEG, PLANT_KNEE_DEG] * 6) * DEG2RAD
    hold_ticks = int(round(_HOLD_S / env.dt))

    vx = vy = om = 0.0
    period = 3.2
    alpha = 0.0
    clampfit = False

    def new_gait() -> "NoSlipGait":
        if clampfit:
            g = NoSlipGait.clamp_fit(vx=vx, vy=vy, omega=om)
        else:
            g = NoSlipGait(period=period, vx=vx, vy=vy, omega=om,
                           alpha=alpha)
        g.sync_plant_stance(PLANT_HIP_DEG, PLANT_KNEE_DEG)
        return g

    env.reset()
    gait = new_gait()
    tick = 0
    msg = ""
    win = "noslip drive"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 960, 720)
    print("noslip drive window open — controls are drawn in the window")

    while True:
        t0 = time.monotonic()
        if tick < hold_ticks:
            act = q_rad_to_action(plant_rad)
        else:
            gait.set_velocity(vx=vx, vy=vy, omega=om)
            t = (tick - hold_ticks) * env.dt
            act = q_rad_to_action(np.asarray(gait.desired_deg(t)) * DEG2RAD)
        _obs, _r, term, trunc, info = env.step(act)
        tick += 1
        if term or trunc:
            msg = f"[{info.get('termination_reason') or 'episode end'}] reset"
            print(msg)
            vx = vy = om = 0.0
            env.reset()
            gait = new_gait()
            tick = 0

        frame = env.render()
        img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        # Root free-joint world-frame linear velocity, rotated into the
        # chassis yaw frame (goal env has no _body_vel_xy helper).
        qw, qx, qy, qz = env.data.xquat[env._chassis_bid]
        yaw = np.arctan2(2 * (qw * qz + qx * qy),
                         1 - 2 * (qy * qy + qz * qz))
        vw = env.data.qvel[0:2]
        cy, sy = np.cos(yaw), np.sin(yaw)
        v = (cy * vw[0] + sy * vw[1], -sy * vw[0] + cy * vw[1])

        spd = float(np.hypot(vx, vy))
        if tick < hold_ticks:
            state = "settling at plant..."
        elif spd < 0.002 and abs(om) < 0.01:
            state = "STOP (quasi-static: holds indefinitely)"
        else:
            parts = []
            if vx > 0.002:
                parts.append("FWD")
            elif vx < -0.002:
                parts.append("REV")
            if vy > 0.002:
                parts.append("STRAFE-L")
            elif vy < -0.002:
                parts.append("STRAFE-R")
            if om > 0.01:
                parts.append("TURN-L")
            elif om < -0.01:
                parts.append("TURN-R")
            state = "+".join(parts) or "…"
        for i, (line, color) in enumerate((
                (f"NO-SLIP GAIT   {state}", (40, 240, 40)),
                (f"CMD vx {vx:+.3f}  vy {vy:+.3f} m/s   omega {om:+.2f} "
                 f"rad/s   (clamps: 0.04/0.035/0.30)", (40, 240, 40)),
                (f"ACTUAL vx {v[0]:+.3f}  vy {v[1]:+.3f} m/s   "
                 f"phase: {gait.phase_name() if tick >= hold_ticks else '-'}",
                 (200, 200, 40)),
                (("CLAMP-FIT preset (7)   1/2/3 or 4/5/6 leaves it"
                  if clampfit else
                  f"period {period:.1f}s (1/2/3)   alpha {alpha:.1f} "
                  f"(4/5/6: 0=step-then-shift, 1=continuous)   "
                  f"7=clamp-fit"), (180, 180, 180)),
                ("I/K fwd-back  J/L strafe  U/O turn  0/space stop  "
                 "R reset  Q quit", (180, 180, 180)))):
            cv2.putText(img, line, (10, 24 + 22 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)
        if msg:
            cv2.putText(img, msg, (10, 140), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 200, 255), 1, cv2.LINE_AA)
        cv2.imshow(win, img)

        k = cv2.waitKeyEx(1)
        if k in (ord("i"), ord("I"), _UP):
            vx += _STEP_V; msg = ""
        elif k in (ord("k"), ord("K"), _DOWN):
            vx -= _STEP_V; msg = ""
        elif k in (ord("j"), ord("J"), _LEFT):
            vy += _STEP_V; msg = ""
        elif k in (ord("l"), ord("L"), _RIGHT):
            vy -= _STEP_V; msg = ""
        elif k in (ord("u"), ord("U")):
            om += _STEP_W; msg = ""
        elif k in (ord("o"), ord("O")):
            om -= _STEP_W; msg = ""
        elif k in (ord("0"), ord(" ")):
            vx = vy = om = 0.0; msg = ""
        elif k in _PERIODS:
            period, clampfit = _PERIODS[k], False
            gait = new_gait()
            tick = max(tick, hold_ticks)  # skip re-hold; gait re-pins feet
            msg = f"period -> {period:.1f}s (gait rebuilt, feet re-pinned)"
        elif k in _ALPHAS:
            alpha, clampfit = _ALPHAS[k], False
            gait = new_gait()
            tick = max(tick, hold_ticks)  # skip re-hold; gait re-pins feet
            msg = f"alpha -> {alpha:.1f} (gait rebuilt, feet re-pinned)"
        elif k == ord("7"):
            clampfit = True
            gait = new_gait()
            tick = max(tick, hold_ticks)  # skip re-hold; gait re-pins feet
            msg = "CLAMP-FIT preset (period 6s, swing 0.40, alpha 1)"
        elif k in (ord("r"), ord("R")):
            vx = vy = om = 0.0
            env.reset()
            gait = new_gait()
            tick = 0
            msg = "manual reset"
        elif k in (ord("q"), ord("Q"), 27):
            break
        if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
            break
        # NoSlipGait clamps internally; mirror for honest HUD numbers.
        vx = float(np.clip(vx, -gait.MAX_VX, gait.MAX_VX))
        vy = float(np.clip(vy, -gait.MAX_VY, gait.MAX_VY))
        om = float(np.clip(om, -gait.MAX_OMEGA, gait.MAX_OMEGA))

        if args.realtime > 0:
            dt = env.dt / args.realtime - (time.monotonic() - t0)
            if dt > 0:
                time.sleep(dt)

    cv2.destroyAllWindows()
    env.close()
    print("closed — clean exit")


if __name__ == "__main__":
    main()
