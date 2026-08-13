"""probe_quad_crawl — can a SCRIPTED open-loop gait walk on four legs?

Spec-cycle tool for the quad track's four-leg-WALKING line (08-13).
The QUADWALK semantics bank needs an honest reference trajectory the
way the WALK bank uses the hardware-proven TripodGait; this probe
holds every scripted rear-four scheme tried so far so the search can
resume without rebuilding it. FINDINGS (08-13, controller CPU, DR
off, quadwalk mode, champion walk stack + quad income):

  scheme=trot      rear-four 2-2 trot (tripod timing minus fronts,
                   mid feet splayed forward): survives with fronts
                   clean but fwd ~0.00 m/15 s — the diagonal 2-leg
                   support phases pivot/slip away all commanded
                   stride. Without the splay it pitch-trips in <1 s
                   (CoM ahead of the 4-foot polygon's front edge).
  scheme=crawl     4-beat crawl, duty 0.75, one swing per quarter
                   (creep order LH,LF,RH,RF = legs 2,1,3,4): the MID
                   legs never lift — when one swings the CoM is
                   ~0.07 m outside the remaining triangle (matches
                   quadruped_feasibility geometry) so the body tips
                   onto the "swinging" foot and pins it. Splay +
                   body-back statics do not fix it.
  scheme=twophase  distance-clock crawl: body advances ONLY while
                   both mids are planted (rear-swing quarters at
                   2*vx), holds + sways (5-6 cm, leading by 1/8
                   cycle) through the mid-swing quarters, lift-first
                   swing profile. Mids finally step, but net body
                   motion is -0.02..-0.10 m (backward!) and the rear
                   legs chatter 2-3x their commanded step count —
                   rocking + slip rectify the cycle backwards.

  CONCLUSION: static 4-leg feasibility (c57 GO) does NOT extend to
  open-loop stepping; the honest quadwalk reference needs either
  video-driven iteration of these schemes on a train pod (this file,
  --video) or closed-loop weight shift — i.e. possibly RL itself,
  which is an MDP_PREFLIGHT chicken-and-egg the operator has to rule
  on. Until then the QUADWALK bank SKIPs and quadwalk arms are
  launch-blocked (test_task_semantics.QUADWALK_REFERENCE_BLOCKED).

Usage (controller or any train pod):
    python3 -m rl_move.sim.probe_quad_crawl --scheme twophase \
        [--vx 0.03] [--period 3.2] [--mid-fwd 0.05] [--body-back 0.05] \
        [--sway 0.06] [--lift 0.03] [--seconds 15] [--seed 0] \
        [--video out.mp4]
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

_RL = Path(__file__).resolve().parents[1]
_PROTO = _RL.parent
for p in (str(_PROTO), str(_PROTO / "linux_control")):
    if p not in sys.path:
        sys.path.insert(0, p)

from rl_move.config import load_config  # noqa: E402
from rl_move.robot_state import DEG2RAD  # noqa: E402

WALK_PLANT = (20.0, 80.0)
TUCK = np.array([0.0, -1.10, 2.40])   # feasibility FRONT_POSES["tuck"]
LIFT_LEGS = (0, 5)
MID_LEGS = (1, 4)
# The stack the quad lineage trains with (cw-quad-hold2 cfg-sets).
STACK = {("reward", "k_step_event"): 1.0,
         ("reward", "k_drag_loaded"): 10.0,
         ("reward", "k_park_duty"): 1.0,
         ("reward", "walk_kernel_prog_gate"): 1.0,
         ("reward", "walk_anchor_gate"): 1.0,
         ("reward", "anchor_tol_mm"): 10.0,
         ("reward", "k_quad_clear"): 1.5,
         ("reward", "k_quad_plant"): 1.0,
         ("goal", "quad_grace_s"): 1.5}


class QuadCrawl:
    """Two-phase 4-beat crawl (scheme=twophase) with crawl/trot
    degenerations; open-loop body-frame foot targets through the same
    leg IK the deployed TripodGait uses."""

    SWING = (2, 1, 3, 4)          # creep order: LH, LF(mid), RH, RF(mid)

    def __init__(self, scheme: str = "twophase", plant=WALK_PLANT,
                 period: float = 3.2, lift: float = 0.03,
                 mid_fwd: float = 0.05, body_back: float = 0.05,
                 sway: float = 0.06):
        from tripod_gait import TripodGait
        self.scheme = scheme
        self.g = TripodGait(vx=0.0, lift=lift)
        self.g.sync_plant_stance(*plant)
        self.period = period
        self.lift = lift
        self.mid_fwd = mid_fwd
        self.body_back = body_back
        self.sway = sway
        self.u = {f: 0.0 for f in (1, 2, 3, 4)}
        self.u_lift: dict = {}
        self._sway_y = 0.0
        self._last_t = None
        self.phase = 0.0

    def desired_rad(self, t: float, vx: float) -> np.ndarray:
        import tripod_gait as TG
        g = self.g
        dt = 0.0 if self._last_t is None else max(0.0, t - self._last_t)
        self._last_t = t
        if vx > 1e-4:
            self.phase = (self.phase + dt / self.period) % 1.0
        k = int(self.phase * 4) % 4
        s = (self.phase * 4) % 1.0
        swing_leg = self.SWING[k]
        two_phase = self.scheme == "twophase"
        moving = (k in (0, 2) or not two_phase) and vx > 1e-4
        dD = ((2.0 if two_phase else 1.0) * vx * dt) if moving else 0.0
        pl = (self.phase + 0.125) % 1.0
        kl = int(pl * 4) % 4
        tgt = -self.sway if kl == 1 else (self.sway if kl == 3 else 0.0)
        a = 1.0 - math.exp(-dt / 0.30) if dt > 0 else 0.0
        self._sway_y += a * (tgt - self._sway_y)
        out = []
        amp = vx * self.period
        A = {1: amp / 2.0, 4: amp / 2.0, 2: amp / 4.0, 3: amp / 4.0}
        for i, ang in enumerate(g.leg_angles):
            if i in LIFT_LEGS:
                out.extend([0.0, 0.0, 0.0])   # caller overrides (tuck)
                continue
            if i == swing_leg and vx > 1e-4:
                if i not in self.u_lift:
                    self.u_lift[i] = self.u[i]
                dz = self.lift * math.sin(math.pi * min(s / 0.95, 1.0))
                if s < 0.30:
                    u = self.u_lift[i]
                else:
                    su = min((s - 0.30) / 0.65, 1.0)
                    u = self.u_lift[i] + (A[i] - self.u_lift[i]) * su
                self.u[i] = u
            else:
                self.u_lift.pop(i, None)
                self.u[i] -= dD
                u = self.u[i]
                dz = 0.0
            off_x = (self.mid_fwd if i in MID_LEGS else 0.0) \
                + self.body_back
            fx = g._foot_radius_eff * math.cos(ang) + u + off_x
            fy = g._foot_radius_eff * math.sin(ang) - self._sway_y
            ox, oy = 0.1 * math.cos(ang), 0.1 * math.sin(ang)
            rx, ry = fx - ox, fy - oy
            ca, sa = math.cos(ang), math.sin(ang)
            x_yaw = ca * rx + sa * ry
            y_yaw = -sa * rx + ca * ry
            ik = TG._leg_ik((math.hypot(x_yaw, y_yaw), 0.0,
                             g.foot_neutral_z + dz))
            if ik is None:
                out.extend([0.0, math.radians(g.plant_hip_deg),
                            math.radians(g.plant_knee_deg)])
            else:
                p, pt = ik
                out.extend([math.atan2(y_yaw, x_yaw), p, pt])
        return np.array(out)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scheme", default="twophase",
                    choices=("trot", "crawl", "twophase"))
    ap.add_argument("--vx", type=float, default=0.03)
    ap.add_argument("--period", type=float, default=3.2)
    ap.add_argument("--mid-fwd", type=float, default=0.05)
    ap.add_argument("--body-back", type=float, default=0.05)
    ap.add_argument("--sway", type=float, default=0.06)
    ap.add_argument("--lift", type=float, default=0.03)
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--video", default=None)
    args = ap.parse_args()

    from rl_move.sim.joint_task import q_rad_to_action
    from rl_move.sim.servo_model import SimServoParams
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv
    from tripod_gait import TripodGait

    cfg = load_config()
    for (sec, leaf), val in STACK.items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=False,
        dr_scale=0.0, episode_seconds=args.seconds, seed=args.seed,
        cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad", "walk", "quadwalk"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "quadwalk" else 0.0)
    env.reset()
    traj = env._goal_traj
    assert traj.mode == "quadwalk"
    hold_n = int(round(2.0 / env.dt))
    ramp_n = int(round(1.0 / env.dt))
    traj.vx[:] = args.vx
    traj.vx[:hold_n] = 0.0
    traj.vx[hold_n:hold_n + ramp_n] = args.vx * np.linspace(0, 1, ramp_n)
    traj.vy[:] = 0.0
    if traj.wz is not None:
        traj.wz[:] = 0.0

    if args.scheme == "trot":
        gait = TripodGait(vx=0.0, lift=args.lift)
        gait.sync_plant_stance(*WALK_PLANT)
        orig = gait._foot_target_in_body

        def patched(i, vx, vy, om):
            dx, dy, dz = orig(i, vx, vy, om)
            if i in MID_LEGS:
                dx += args.mid_fwd
            return (dx, dy, dz)
        gait._foot_target_in_body = patched
        gait.reset_phase()

        def q_of(t, vx):
            gait.set_velocity(vx=vx, vy=0.0)
            return np.asarray(gait.desired_deg(t)) * DEG2RAD
    else:
        qc = QuadCrawl(scheme=args.scheme, period=args.period,
                       lift=args.lift, mid_fwd=args.mid_fwd,
                       body_back=args.body_back, sway=args.sway)

        def q_of(t, vx):
            return qc.desired_rad(t, vx)

    plant = np.array([0.0, *WALK_PLANT] * 6) * DEG2RAD
    frames = []
    step, tot, term = 0, 0.0, False
    x0 = env.data.xpos[env._chassis_bid, 0]
    lc = lt = 0
    n_skip = int(round(3.0 / env.dt))
    sw = [0] * 6
    prev = [True] * 6
    while True:
        t = step * env.dt
        i = min(step, len(traj.vx) - 1)
        q = q_of(t, float(traj.vx[i]))
        blend = min(t / 1.5, 1.0)
        for leg in LIFT_LEGS:
            q[3 * leg:3 * leg + 3] = ((1 - blend)
                                      * plant[3 * leg:3 * leg + 3]
                                      + blend * TUCK)
        _obs, r, term, trunc, info = env.step(q_rad_to_action(q))
        con = [float(env.data.sensordata[env._touch_adr[f]]) > 0.5
               for f in range(6)]
        for f in range(6):
            if prev[f] and not con[f]:
                sw[f] += 1
            prev[f] = con[f]
        tot += float(r)
        if step >= n_skip:
            lt += 1
            lc += sum(1 for leg in LIFT_LEGS if con[leg])
        if args.video:
            fr = env.render()
            if fr is not None:
                frames.append(fr)
        step += 1
        if term:
            print("TERMINATED:", info.get("termination_reason"))
        if term or trunc:
            break
    fwd = env.data.xpos[env._chassis_bid, 0] - x0
    print(f"scheme={args.scheme} vx={args.vx} period={args.period} "
          f"mid_fwd={args.mid_fwd} body_back={args.body_back} "
          f"sway={args.sway} lift={args.lift} seed={args.seed}")
    print(f"return={tot:.1f} steps={step} fwd={fwd:+.3f} m "
          f"lift_duty_tail={lc / max(2 * lt, 1):.2f} term={term} "
          f"swings={sw}")
    if args.video and frames:
        import imageio
        imageio.mimsave(args.video, frames, fps=int(round(1 / env.dt)))
        print("video ->", args.video)
    env.close()


if __name__ == "__main__":
    main()
