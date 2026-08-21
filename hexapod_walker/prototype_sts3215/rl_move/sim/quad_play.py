#!/usr/bin/env python3
"""Interactive MuJoCo playground for the tip-back QUAD walk.

    ./sim_quad.sh          # from prototype_sts3215/sim_viewer/
    python rl_move/sim/quad_play.py   # plain python, NOT mjpython (cv2)

Drives the EXACT gait module the robot runs (motor_setup/quad_walk.py —
the ``quad_walk`` demo behind the webui Quad tab) through the fitted
servo twin, but as a live state machine instead of a fixed timeline:

    PLANT --7/W--> ENTRY (shift back, mids step out, tuck, rear up)
    REARED --W--> WALK --Space--> (freeze at next all-4-stance) REARED
    REARED/WALK --8--> EXIT (regather, re-step mids, level, untuck)
    --> PLANT

Keys are drawn IN the window (cv2 owns every key — no viewer toggles):
    7 / W   rear up (W also auto-starts walking once reared)
    T       gait: WALK (lateral sequence, one foot up, statically
            stable) or TROT (diagonal pairs like a horse, ~2x pace —
            sim-swept 08-18, never falls, rocks LESS than the walk).
            Takes effect at the next rear-up.
    Space   stop walking — freezes at the next all-4-feet-down window
    8       sit back down (auto-stops the walk first)
    9       reset the sim to the plant pose
    - / =   gait speed 0.25x .. 2.0x (LIVE, same knob as the webui)
    P       shove the chassis forward (4 N for 0.3 s — robustness poke)
    Z / X   camera zoom in / out;  mouse drag = orbit
    Q/Esc   quit
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import numpy as np

_SIM = Path(__file__).resolve().parent
_PROTO = _SIM.parents[1]
for _p in (_PROTO, _PROTO / "motor_setup", _PROTO / "linux_control",
           str(_SIM)):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import cv2  # noqa: E402
import mujoco  # noqa: E402

from eval_dances import (CTRL_HZ, ROBOT_PLANT_DEG, STREAM_ACC_UNITS,  # noqa: E402
                         clip_limits, place_at_plant, up_z)
from rl_move.robot_state import DEG2RAD  # noqa: E402
from rl_move.sim.servo_model import (SIM_MODEL_PATH, ServoProfile,  # noqa: E402
                                     SimServoParams, apply_params_to_model,
                                     build_model, joint_qpos_addrs,
                                     position_actuator_ids)
import quad_walk as QW  # noqa: E402

WIN = "quad_play — tip-back walk (7 rear · W walk · Space stop · 8 sit)"


def all_stance(g: "QW.QuadRearWalk", tw: float) -> bool:
    """True when all 4 support feet are planted at walk-clock ``tw``."""
    return g.walk_all_stance_at(tw)


class Player:
    PLANT, ENTRY, REARED, WALK, STOPPING, EXIT = range(6)
    NAMES = {PLANT: "PLANT (press 7 or W to rear up)",
             ENTRY: "REARING UP…", REARED: "REARED (W = walk, 8 = sit)",
             WALK: "WALKING (Space = stop, 8 = sit)",
             STOPPING: "stopping — finishing the step…",
             EXIT: "SITTING BACK DOWN…"}

    # Profile speed: the demo streamer (PoseStreamer) writes per-joint
    # speeds up to 3000 counts/s; the fitted default (350 ≈ 31 deg/s)
    # is the system-ID *test* speed and makes every gait a shuffle.
    # Match verify_noslip's hardware write profile (1500 counts/s).
    VEL_SCALE = 1500.0 / 350.0

    def __init__(self) -> None:
        self.model = build_model(mesh_visuals=False, flat_terrain=True)
        self.params = SimServoParams.load(SIM_MODEL_PATH)
        apply_params_to_model(self.model, self.params)
        self.data = mujoco.MjData(self.model)
        self.qadr = joint_qpos_addrs(self.model)
        self.pos_act = position_actuator_ids(self.model)
        self.chassis = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        self.vadr = np.array([self.model.jnt_dofadr[mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, f"L{i}_{ax}")]
            for i in range(6) for ax in ("yaw", "pitch", "knee")])
        self.q_plant = clip_limits(np.asarray(ROBOT_PLANT_DEG) * DEG2RAD)
        self.h = self.model.opt.timestep
        self.sub = max(1, int(round(1.0 / CTRL_HZ / self.h)))
        self.speed = 1.0
        self.gait_name = "walk"
        self.reset()

    # ---- sim plumbing ------------------------------------------------
    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        place_at_plant(mujoco, self.model, self.data, self.qadr,
                       self.pos_act, self.q_plant)
        self.profile = ServoProfile(self.params, self.q_plant,
                                    vel_scale=self.VEL_SCALE)
        self.gait = QW.QuadRearWalk(list(ROBOT_PLANT_DEG), 1e6,
                                    gait=self.gait_name)
        self.state = self.PLANT
        self.t = 0.0            # entry clock
        self.tw = 0.0           # walk clock
        self.exit_fn = None
        self.tx = 0.0           # exit clock (absolute t into exit_fn)
        self.walk_queued = False
        self.push_left = 0.0
        self.x0 = float(self.data.qpos[0])
        self.peak_cur = 0.0

    def _pose(self) -> list[float]:
        g = self.gait
        if self.state == self.PLANT:
            return list(ROBOT_PLANT_DEG)
        if self.state == self.ENTRY:
            return g.pose_at(min(self.t, QW.ENTRY_TOTAL_S - 1e-4))
        if self.state in (self.REARED, self.WALK, self.STOPPING):
            if self.tw <= 0.0:
                return g.pose_at(QW.ENTRY_TOTAL_S - 1e-4)
            return g.pose_at(QW.ENTRY_TOTAL_S + self.tw)
        return self.exit_fn(self.tx)

    @property
    def speed_eff(self) -> float:
        """Live speed with the gait's cap (the calm trot survives 0.5-2x
        in sim; the cap is prudence for hardware's two-foot beats)."""
        cap = QW.GAITS[self.gait_name].get("speed_cap", 2.0)
        return min(self.speed, cap)

    def step(self) -> None:
        dt = 1.0 / CTRL_HZ
        dg = dt * self.speed_eff
        if self.state == self.ENTRY:
            self.t += dg
            if self.t >= QW.ENTRY_TOTAL_S:
                self.state = self.WALK if self.walk_queued else self.REARED
                self.walk_queued = False
        elif self.state in (self.WALK, self.STOPPING):
            self.tw += dg
            if self.state == self.STOPPING and all_stance(self.gait, self.tw):
                self.state = self.REARED
        elif self.state == self.EXIT:
            self.tx += dg
            if self.tx >= self.exit_fn_end:
                self.state = self.PLANT

        q = clip_limits(np.asarray(self._pose()) * DEG2RAD)
        self.profile.command(q, acc_units=STREAM_ACC_UNITS)
        db = self.profile.deadband_rad
        for _ in range(self.sub):
            tgt = self.profile.tick(self.h)
            qj = self.data.qpos[self.qadr]
            err = tgt - qj
            self.data.ctrl[self.pos_act] = qj + np.sign(err) * np.maximum(
                np.abs(err) - db, 0.0)
            if self.push_left > 0.0:
                self.data.xfrc_applied[self.chassis][0] = 4.0
            else:
                self.data.xfrc_applied[self.chassis][0] = 0.0
            mujoco.mj_step(self.model, self.data)
        self.push_left = max(0.0, self.push_left - dt)
        cur = float(np.max(np.abs(
            self.data.qfrc_actuator[self.vadr]))) * 1.2
        self.peak_cur = max(self.peak_cur, cur)
        self.cur = cur

    # ---- commands ----------------------------------------------------
    def cmd_rear(self, then_walk: bool = False) -> None:
        if self.state == self.PLANT:
            self.gait = QW.QuadRearWalk(list(ROBOT_PLANT_DEG), 1e6,
                                        gait=self.gait_name)
            self.t, self.tw = 0.0, 0.0
            self.state = self.ENTRY
            self.walk_queued = then_walk
        elif self.state == self.REARED and then_walk:
            self.state = self.WALK
        elif self.state == self.ENTRY and then_walk:
            self.walk_queued = True

    def cmd_stop(self) -> None:
        if self.state == self.WALK:
            self.state = (self.REARED if all_stance(self.gait, self.tw)
                          else self.STOPPING)
        self.walk_queued = False

    def cmd_sit(self) -> None:
        if self.state not in (self.REARED, self.WALK, self.STOPPING):
            return
        # exit math needs an all-stance freeze; close enough is fine —
        # the regather blend in exit phase 1 absorbs a mid-swing foot.
        tw = self.tw
        secs = QW.ENTRY_TOTAL_S + tw + QW.EXIT_TOTAL_S
        self.exit_fn = QW.QuadRearWalk(list(ROBOT_PLANT_DEG), secs,
                                       gait=self.gait_name).pose_at
        self.tx = QW.ENTRY_TOTAL_S + tw
        self.exit_fn_end = secs
        self.state = self.EXIT

    def cmd_gait(self) -> None:
        """Toggle walk/trot. Mid-run the clocks and foot schedules of the
        two gaits don't line up, so it takes effect at the next rear-up."""
        names = list(QW.GAITS)
        self.gait_name = names[(names.index(self.gait_name) + 1)
                               % len(names)]


def main() -> None:
    pl = Player()
    renderer = mujoco.Renderer(pl.model, height=600, width=960)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance, cam.azimuth, cam.elevation = 0.62, 145.0, -18.0
    drag = {"on": False, "x": 0, "y": 0}

    def on_mouse(event, x, y, flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            drag.update(on=True, x=x, y=y)
        elif event == cv2.EVENT_LBUTTONUP:
            drag["on"] = False
        elif event == cv2.EVENT_MOUSEMOVE and drag["on"]:
            cam.azimuth -= 0.4 * (x - drag["x"])
            cam.elevation = float(np.clip(
                cam.elevation - 0.3 * (y - drag["y"]), -85.0, 5.0))
            drag.update(x=x, y=y)

    cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WIN, on_mouse)

    help_lines = ["7/W rear up   W walk   Space stop   8 sit   9 reset",
                  "T gait walk/trot   -/= speed   P shove   Z/X zoom   "
                  "drag orbit   Q quit"]
    next_t = time.monotonic()
    while True:
        pl.step()
        cam.lookat[:] = (float(pl.data.qpos[0]), float(pl.data.qpos[1]),
                         0.10)
        renderer.update_scene(pl.data, camera=cam)
        img = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)

        uz = up_z(pl.data, pl.chassis)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, uz))))
        dist = 1000.0 * (float(pl.data.qpos[0]) - pl.x0)
        pending = ("" if pl.gait.phase == QW.GAITS[pl.gait_name]["phase"]
                   or pl.state == Player.PLANT else " (next rear-up)")
        capped = (f" (capped {pl.speed_eff:.2f})"
                  if pl.speed_eff < pl.speed else "")
        hud = [Player.NAMES[pl.state],
               f"gait {pl.gait_name.upper()}{pending}   "
               f"speed {pl.speed:.2f}x{capped}   walked {dist:+.0f} mm   "
               f"tilt {tilt:.0f} deg   cur {pl.cur:.1f} A "
               f"(peak {pl.peak_cur:.1f})"]
        if uz < 0.55:
            hud[0] = "FELL OVER — press 9 to reset"
        for i, line in enumerate(hud):
            cv2.putText(img, line, (12, 28 + 26 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                        (60, 240, 120) if i == 0 else (230, 230, 230), 2)
        for i, line in enumerate(help_lines):
            cv2.putText(img, line, (12, img.shape[0] - 34 + 22 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (170, 170, 170), 1)
        cv2.imshow(WIN, img)

        k = cv2.waitKeyEx(1)
        if k in (ord("q"), ord("Q"), 27):
            break
        elif k in (ord("7"),):
            pl.cmd_rear(then_walk=False)
        elif k in (ord("w"), ord("W"), 63232, 2490368):  # W / Up arrow
            pl.cmd_rear(then_walk=True)
        elif k == ord(" "):
            pl.cmd_stop()
        elif k == ord("8"):
            pl.cmd_sit()
        elif k == ord("9"):
            pl.reset()
        elif k in (ord("-"), ord("_")):
            pl.speed = max(0.25, round(pl.speed - 0.25, 2))
        elif k in (ord("="), ord("+")):
            pl.speed = min(2.0, round(pl.speed + 0.25, 2))
        elif k in (ord("t"), ord("T")):
            pl.cmd_gait()
        elif k in (ord("p"), ord("P")):
            pl.push_left = 0.3
        elif k in (ord("z"), ord("Z")):
            cam.distance = max(0.25, cam.distance * 0.85)
        elif k in (ord("x"), ord("X")):
            cam.distance = min(2.5, cam.distance / 0.85)

        next_t += 1.0 / CTRL_HZ
        lag = next_t - time.monotonic()
        if lag > 0:
            time.sleep(lag)
        else:
            next_t = time.monotonic()

    cv2.destroyAllWindows()
    renderer.close()


if __name__ == "__main__":
    main()
