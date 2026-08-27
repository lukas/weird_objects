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

import argparse
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

import mujoco  # noqa: E402

from eval_dances import (CTRL_HZ, STREAM_ACC_UNITS,  # noqa: E402
                         clip_limits, place_at_plant, up_z)
from rl_move.joint_frame import (  # noqa: E402
    robot_abs_deg_to_sim_rad, robot_stand_degrees,
)
from rl_move.sim.servo_model import (SIM_MODEL_PATH, ServoProfile,  # noqa: E402
                                     SimServoParams, apply_params_to_model,
                                     build_model, joint_qpos_addrs,
                                     position_actuator_ids)
import quad_walk as QW  # noqa: E402

WIN = "quad_play — tip-back walk (7 rear · W walk · Space stop · 8 sit)"


def all_stance(g: "QW.QuadRearWalk", tw: float) -> bool:
    """True when all 4 support feet are planted at walk-clock ``tw``."""
    return g.walk_all_stance_at(tw)


def apply_plant_override(pl: "Player", *,
                         hip_deg: float | None = None,
                         knee_deg: float | None = None) -> None:
    """Override the learned stand pose for hardware-regression sweeps."""
    if hip_deg is None and knee_deg is None:
        return
    plant = list(pl.robot_plant_deg)
    for leg in range(6):
        if hip_deg is not None:
            plant[3 * leg + 1] = float(hip_deg)
        if knee_deg is not None:
            plant[3 * leg + 2] = float(knee_deg)
    pl.robot_plant_deg = plant
    pl.q_plant = clip_limits(robot_abs_deg_to_sim_rad(plant))


def contact_snapshot(pl: "Player") -> tuple[set[str], set[str]]:
    """Return (terrain-contacting legs, non-foot terrain contacts)."""
    feet: set[str] = set()
    bad: set[str] = set()
    for i in range(pl.data.ncon):
        con = pl.data.contact[i]
        names = [
            mujoco.mj_id2name(pl.model, mujoco.mjtObj.mjOBJ_GEOM, int(g))
            or f"geom#{int(g)}"
            for g in (con.geom1, con.geom2)
        ]
        if "terrain" not in names:
            continue
        other = names[1] if names[0] == "terrain" else names[0]
        if other.endswith("_foot"):
            feet.add(other.split("_", 1)[0])
        else:
            bad.add(other)
    return feet, bad


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
        self.robot_plant_deg = robot_stand_degrees()
        self.q_plant = clip_limits(
            robot_abs_deg_to_sim_rad(self.robot_plant_deg))
        self.h = self.model.opt.timestep
        self.sub = max(1, int(round(1.0 / CTRL_HZ / self.h)))
        self.speed = 1.0
        self.direction = 1.0
        self.gait_name = "walk"
        self.reset()

    # ---- sim plumbing ------------------------------------------------
    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        place_at_plant(mujoco, self.model, self.data, self.qadr,
                       self.pos_act, self.q_plant)
        self.profile = ServoProfile(self.params, self.q_plant,
                                    vel_scale=self.VEL_SCALE)
        self.gait = QW.QuadRearWalk(list(self.robot_plant_deg), 1e6,
                                    gait=self.gait_name,
                                    direction=self.direction)
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
            return list(self.robot_plant_deg)
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

        q = clip_limits(robot_abs_deg_to_sim_rad(self._pose()))
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
            self.gait = QW.QuadRearWalk(list(self.robot_plant_deg), 1e6,
                                        gait=self.gait_name,
                                        direction=self.direction)
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
        self.exit_fn = QW.QuadRearWalk(list(self.robot_plant_deg), secs,
                                       gait=self.gait_name,
                                       direction=self.direction).pose_at
        self.tx = QW.ENTRY_TOTAL_S + tw
        self.exit_fn_end = secs
        self.state = self.EXIT

    def cmd_gait(self) -> None:
        """Toggle walk/trot. Mid-run the clocks and foot schedules of the
        two gaits don't line up, so it takes effect at the next rear-up."""
        names = list(QW.GAITS)
        self.gait_name = names[(names.index(self.gait_name) + 1)
                               % len(names)]


def run_headless(gaits: list[str], seconds: float, *,
                 direction: float = 1.0,
                 plant_hip: float | None = None,
                 plant_knee: float | None = None) -> int:
    min_walk_progress_mm = 25.0
    failures = 0
    for gait in gaits:
        pl = Player()
        pl.gait_name = gait
        pl.direction = -1.0 if float(direction) < 0.0 else 1.0
        apply_plant_override(pl, hip_deg=plant_hip, knee_deg=plant_knee)
        pl.reset()
        rear_only = gait.startswith("rear")
        pl.cmd_rear(then_walk=not rear_only)
        # Headless uses wall time, while quad entry is slowed by each gait's
        # hardware speed cap. Sim long enough to actually reach reared/walk;
        # otherwise a too-short run can report "no contacts" as success.
        min_demo_after_entry = 1.5 if rear_only else 4.0
        min_wall = (
            (QW.ENTRY_TOTAL_S + min_demo_after_entry)
            / max(pl.speed_eff, 0.05)
        )
        n = int(max(18.0 if rear_only else seconds, min_wall) * CTRL_HZ)
        max_tilt = 0.0
        fell = False
        # Derive leg sets from the gait module itself — the 08-22
        # front/support rotation (7c745a01: FRONT_LEGS=(2,3),
        # SUPPORT_LEGS=(0,1,4,5)) made the old hardcoded
        # {L1..L4}/{L0,L5} sets report false support/front failures.
        support_legs = {f"L{i}" for i in QW.SUPPORT_LEGS}
        front_legs = {f"L{i}" for i in QW.FRONT_LEGS}
        front_hits: set[str] = set()
        bad_contacts: set[str] = set()
        support_min = 99
        support_final: set[str] = set()
        contact_samples = 0
        walk_x0: float | None = None
        for _ in range(n):
            prev_state = pl.state
            pl.step()
            if (not rear_only and walk_x0 is None
                    and prev_state == pl.ENTRY and pl.state == pl.WALK):
                walk_x0 = float(pl.data.qpos[0])
            uz = up_z(pl.data, pl.chassis)
            max_tilt = max(max_tilt, math.degrees(math.acos(
                max(-1.0, min(1.0, uz)))))
            fell = fell or uz < 0.55 or float(pl.data.qpos[2]) < 0.05
            if pl.state in (pl.REARED, pl.WALK, pl.STOPPING):
                feet, bad = contact_snapshot(pl)
                support_final = feet & support_legs
                support_min = min(support_min, len(support_final))
                front_hits.update(feet & front_legs)
                bad_contacts.update(bad)
                contact_samples += 1
        dist_mm = 1000.0 * (float(pl.data.qpos[0]) - pl.x0)
        walk_dist_mm = None
        if not rear_only:
            if walk_x0 is None:
                walk_x0 = pl.x0
            walk_dist_mm = 1000.0 * (float(pl.data.qpos[0]) - walk_x0)
        expected_min = 4 if rear_only else 2
        support_ok = contact_samples > 0 and support_min >= expected_min
        progress_ok = (
            True if rear_only
            else walk_dist_mm is not None
            and pl.direction * walk_dist_mm >= min_walk_progress_mm
        )
        failed = (
            fell or bool(front_hits) or bool(bad_contacts)
            or not support_ok or not progress_ok
        )
        failures += int(failed)
        contact_note = (
            f"support={support_min if contact_samples else 0}"
            f"/{','.join(sorted(support_final)) or '-'}"
            f" front={','.join(sorted(front_hits)) or '-'}"
            f" scrape={','.join(sorted(bad_contacts)) or '-'}")
        progress_note = (
            "" if rear_only else
            f" walk={walk_dist_mm:+7.1f} mm "
            f"progress={int(progress_ok)} ")
        print(
            f"{gait:16s} dist={dist_mm:+7.1f} mm "
            f"{progress_note}"
            f"max_tilt={max_tilt:5.1f} deg "
            f"peak_cur={pl.peak_cur:4.1f} A "
            f"fell={int(fell)} bad={int(failed)} "
            f"state={Player.NAMES[pl.state].split()[0]} {contact_note}",
            flush=True,
        )
    return 1 if failures else 0


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--headless", action="store_true",
                    help="run a fast non-rendered quad smoke test")
    ap.add_argument("--gaits",
                    default=(
                        "rear,walk,trot,"
                        "rear_safe,walk_safe,trot_safe,"
                        "rear_pitch,walk_pitch,trot_pitch,"
                        "rear_aft,walk_aft,trot_aft,"
                        "rear_high,walk_high,trot_high,"
                        "rear_step,walk_step,trot_step,"
                        "rear_aggressive,walk_aggressive,trot_aggressive"),
                    help="comma-separated gait list for --headless")
    ap.add_argument("--seconds", type=float, default=35.0)
    ap.add_argument("--direction", type=float, default=1.0,
                    help="walk direction for --headless; negative = backward")
    ap.add_argument("--plant-hip", type=float, default=None,
                    help="override stand hip degrees for --headless")
    ap.add_argument("--plant-knee", type=float, default=None,
                    help="override stand knee degrees for --headless")
    args = ap.parse_args()
    if args.headless:
        gaits = [s.strip() for s in args.gaits.split(",") if s.strip()]
        raise SystemExit(run_headless(
            gaits, args.seconds,
            direction=args.direction,
            plant_hip=args.plant_hip, plant_knee=args.plant_knee))

    import cv2  # noqa: PLC0415
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
