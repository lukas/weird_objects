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

  CONCLUSION (superseded — see below): static 4-leg feasibility
  (c57 GO) does NOT extend to open-loop stepping.

  RESOLVED 08-13 (train-0 diag session, this file --diag): a
  STATICALLY-STABLE open-loop quad crawl with both fronts lifted is
  GEOMETRICALLY INFEASIBLE on this robot. Measured, across 14
  configs spanning every physical lever (mid_fwd/body_back
  translation, reach-preserving hip rotation --rot-mid/--rot-rear,
  stance pitch --pitch-deg [env tilt_pitch terminates >=10 deg],
  --front-yaw-deg tuck sweep, adaptive stance-centroid shimmy
  --shift-gain/--shim-tau/--shim-lead at caps up to 0.07, periods
  3.2-6.4 s]:
   - mid-swing support margin: -33..-70 mm in EVERY config (rear
     swings are fine, +35..+55 mm) — the swinging mid is pinned
     (pinned_frac 0.65-1.0) and the body tips onto it (tilt_pitch
     when slow); the tip + recovery rectifies the cycle BACKWARD.
   - the CoM sits +3.3..+5.9 cm AHEAD of the best achievable 4-foot
     centroid; commanded body x-shift does NOT physically realize
     (hip yaw saturated: mids need ~53 deg vs the ±35 deg limit),
     lateral realizes ~30%.
   - geometry: the mid-swing diagonal (mid foot <-> opposite rear)
     crosses the centerline at ~-5 cm behind body center INVARIANT
     of extension (the ±35 deg yaw ray from the mid hip fixes
     dy/dx), while the CoM would need to reach ~-5..-7 cm x or
     ~9-13 cm laterally — both outside the reachable body-shift
     envelope. No foot placement within joint limits fixes this.
  => Only DYNAMIC (closed-loop) balance can walk this robot on four
  legs; an honest scripted open-loop bank reference CANNOT EXIST.
  The QUADWALK bank stays SKIPped
  (test_task_semantics.QUADWALK_REFERENCE_BLOCKED) pending the
  operator ruling on a feedback/RL-derived reference (quad/STATUS
  route 2) — that ruling is now the ONLY route.

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
                 sway: float = 0.06, shift_gain: float = 0.0,
                 shift_cap: float = 0.08, advance: str = "two"):
        from tripod_gait import TripodGait
        self.scheme = scheme
        self.g = TripodGait(vx=0.0, lift=lift)
        self.g.sync_plant_stance(*plant)
        self.period = period
        self.lift = lift
        self.mid_fwd = mid_fwd
        self.body_back = body_back
        self.sway = sway
        # shift_gain > 0: ADAPTIVE 2-D weight shift (replaces the fixed
        # lateral sway): aim the body at the centroid of the three
        # commanded stance feet of the phase-led quarter. With the
        # fronts tucked, EVERY swing lifts a corner of the 4-foot
        # rectangle, so every quarter needs its own CoM target — the
        # fixed sway only served the mid swings and the static
        # body_back actively hurt the rear swings.
        self.shift_gain = shift_gain
        self.shift_cap = shift_cap
        self.advance = advance
        # Stance re-centering (08-13 video/diag session): the support
        # trapezoid of {mids, rears} is centered ~5-8 cm BEHIND the
        # CoM, so the mid-swing triangles are unreachable (margin
        # ~-60 mm) and translation-based offsets (mid_fwd/body_back)
        # saturate the ±35° hip-yaw limit silently. rot_* rotate the
        # stance direction about each HIP (reach-preserving, yaw-
        # budget-aware); pitch_deg tilts the stance plane (nose-up =
        # CoM back) via per-leg foot depth.
        self.rot_mid = 0.0
        self.rot_rear = 0.0
        self.pitch_deg = 0.0
        self.shift_tau = 0.30
        self.shift_lead = 0.125
        self.u = {f: 0.0 for f in (1, 2, 3, 4)}
        self.u_lift: dict = {}
        self._sway_y = 0.0
        self._shift = np.zeros(2)
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
        moving = (k in (0, 2) or not two_phase
                  or self.advance == "all") and vx > 1e-4
        dD = ((2.0 if two_phase and self.advance == "two" else 1.0)
              * vx * dt) if moving else 0.0
        pl = (self.phase + self.shift_lead) % 1.0
        kl = int(pl * 4) % 4
        a = 1.0 - math.exp(-dt / self.shift_tau) if dt > 0 else 0.0
        out = []
        amp = vx * self.period
        A = {1: amp / 2.0, 4: amp / 2.0, 2: amp / 4.0, 3: amp / 4.0}
        # First pass: body-frame foot targets before any body shift.
        raw = {}
        for i, ang in enumerate(g.leg_angles):
            if i in LIFT_LEGS:
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
            s_ang = math.atan2(math.sin(ang), math.cos(ang))
            rot = math.radians(self.rot_mid if i in MID_LEGS
                               else self.rot_rear)
            phi = s_ang - math.copysign(rot, s_ang)
            hx, hy = 0.1 * math.cos(ang), 0.1 * math.sin(ang)
            d = g._foot_radius_eff - 0.1
            fx = hx + d * math.cos(phi) + u + off_x
            fy = hy + d * math.sin(phi)
            # stance-plane pitch: nose-up tilts feet about the y axis
            dz += -math.tan(math.radians(self.pitch_deg)) * fx
            raw[i] = (fx, fy, dz)
        # Body shift: adaptive stance-centroid shimmy, or legacy sway.
        if self.shift_gain > 1e-6 and vx > 1e-4:
            led_swing = self.SWING[kl]
            pts = [(fx, fy) for i, (fx, fy, _) in raw.items()
                   if i != led_swing]
            cent = np.mean(np.array(pts), axis=0)
            tgt2 = np.clip(self.shift_gain * cent,
                           -self.shift_cap, self.shift_cap)
        else:
            sw_y = -self.sway if kl == 1 else \
                (self.sway if kl == 3 else 0.0)
            tgt2 = np.array([0.0, sw_y]) if self.shift_gain <= 1e-6 \
                else np.zeros(2)
        self._shift += a * (tgt2 - self._shift)
        for i, ang in enumerate(g.leg_angles):
            if i in LIFT_LEGS:
                out.extend([0.0, 0.0, 0.0])   # caller overrides (tuck)
                continue
            fx, fy, dz = raw[i]
            # moving the body +shift == moving every foot -shift
            fx -= self._shift[0]
            fy -= self._shift[1]
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
    ap.add_argument("--shift-gain", type=float, default=0.0,
                    help="adaptive stance-centroid weight shift "
                         "(replaces --sway when > 0)")
    ap.add_argument("--shift-cap", type=float, default=0.08)
    ap.add_argument("--rot-mid", type=float, default=0.0,
                    help="deg: rotate mid stance feet about their hips "
                         "toward the front (reach-preserving; yaw "
                         "budget is ±35)")
    ap.add_argument("--rot-rear", type=float, default=0.0)
    ap.add_argument("--pitch-deg", type=float, default=0.0,
                    help="stance-plane pitch, +nose-up (CoM back); "
                         "NOTE env tilt_pitch termination caps this "
                         "at a few degrees")
    ap.add_argument("--shim-tau", type=float, default=0.30,
                    help="weight-shift smoothing time constant, s")
    ap.add_argument("--shim-lead", type=float, default=0.125,
                    help="weight-shift phase lead, cycle fraction")
    ap.add_argument("--front-yaw-deg", type=float, default=0.0,
                    help="yaw the TUCKED fronts outward/back (mass + "
                         "clearance)")
    ap.add_argument("--advance", default="two", choices=("two", "all"),
                    help="body advance only in rear-swing quarters "
                         "(two) or continuously (all)")
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--video", default=None)
    ap.add_argument("--diag", action="store_true",
                    help="per-quarter table: net body dx, swing-foot "
                         "pinned fraction, contact breaks per leg")
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
        render_mode="rgb_array" if args.video else None,
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
                       body_back=args.body_back, sway=args.sway,
                       shift_gain=args.shift_gain,
                       shift_cap=args.shift_cap, advance=args.advance)
        qc.rot_mid = args.rot_mid
        qc.rot_rear = args.rot_rear
        qc.pitch_deg = args.pitch_deg
        qc.shift_tau = args.shim_tau
        qc.shift_lead = args.shim_lead

        def q_of(t, vx):
            return qc.desired_rad(t, vx)

    plant = np.array([0.0, *WALK_PLANT] * 6) * DEG2RAD
    fy_sign = {0: 1.0, 5: -1.0}   # leg0 mount +30 deg, leg5 -30 deg
    tuck_q = {leg: np.array([fy_sign[leg]
                             * math.radians(args.front_yaw_deg),
                             TUCK[1], TUCK[2]]) for leg in LIFT_LEGS}
    frames = []
    step, tot, term = 0, 0.0, False
    x0 = env.data.xpos[env._chassis_bid, 0]
    lc = lt = 0
    n_skip = int(round(3.0 / env.dt))
    sw = [0] * 6
    prev = [True] * 6
    diag_on = args.diag and args.scheme != "trot"
    dxq = [0.0] * 4
    pin_hit = [0] * 4
    pin_tot = [0] * 4
    brk = [[0] * 4 for _ in range(6)]
    marg = [[] for _ in range(4)]     # CoM margin vs 3-foot triangle
    shcmd = [[] for _ in range(4)]    # commanded body shift (x, y)
    comb = [[] for _ in range(4)]     # achieved CoM rel 4-foot centroid
    x_prev = x0
    while True:
        t = step * env.dt
        i = min(step, len(traj.vx) - 1)
        q = q_of(t, float(traj.vx[i]))
        blend = min(t / 1.5, 1.0)
        for leg in LIFT_LEGS:
            q[3 * leg:3 * leg + 3] = ((1 - blend)
                                      * plant[3 * leg:3 * leg + 3]
                                      + blend * tuck_q[leg])
        _obs, r, term, trunc, info = env.step(q_rad_to_action(q))
        con = [float(env.data.sensordata[env._touch_adr[f]]) > 0.5
               for f in range(6)]
        kq = int(qc.phase * 4) % 4 if diag_on else 0
        sq = (qc.phase * 4) % 1.0 if diag_on else 0.0
        for f in range(6):
            if prev[f] and not con[f]:
                sw[f] += 1
                if diag_on and step >= n_skip:
                    brk[f][kq] += 1
            prev[f] = con[f]
        if diag_on:
            x_now = float(env.data.xpos[env._chassis_bid, 0])
            if step >= n_skip:
                dxq[kq] += x_now - x_prev
                if 0.35 <= sq <= 0.95:
                    pin_tot[kq] += 1
                    pin_hit[kq] += int(con[QuadCrawl.SWING[kq]])
                    from rl_move.sim.sim_env import support_margin_m
                    stance = [f for f in (1, 2, 3, 4)
                              if f != QuadCrawl.SWING[kq]]
                    feet_xy = np.array(
                        [env.data.xpos[env._pad_bids[f], :2]
                         for f in stance])
                    marg[kq].append(support_margin_m(
                        feet_xy, env.data.subtree_com[0, :2]) * 1000.0)
                    shcmd[kq].append(qc._shift.copy())
                    # achieved CoM relative to the 4-foot centroid,
                    # rotated into the body frame (x fwd, y left)
                    R = env.data.xmat[env._chassis_bid].reshape(3, 3)
                    all4 = np.array(
                        [env.data.xpos[env._pad_bids[f], :2]
                         for f in (1, 2, 3, 4)])
                    dcom = (env.data.subtree_com[0, :2]
                            - all4.mean(axis=0))
                    comb[kq].append(R[:2, :2].T @ dcom)
            x_prev = x_now
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
          f"swings={sw} shift_gain={args.shift_gain} "
          f"advance={args.advance}")
    if diag_on:
        names = {0: "LH(2)", 1: "LF(1)", 2: "RH(3)", 3: "RF(4)"}
        print("quarter  swing  net_dx_m  pinned_frac  "
              "com_margin_mm(med)  shift_cmd_xy  com_ach_xy  "
              "breaks/leg(0..5)")
        for kq in range(4):
            pf = pin_hit[kq] / max(pin_tot[kq], 1)
            bl = [brk[f][kq] for f in range(6)]
            mm = float(np.median(marg[kq])) if marg[kq] else float("nan")
            sh = (np.median(np.array(shcmd[kq]), axis=0)
                  if shcmd[kq] else np.zeros(2))
            cb = (np.median(np.array(comb[kq]), axis=0)
                  if comb[kq] else np.zeros(2))
            print(f"  k={kq}    {names[kq]:6s} {dxq[kq]:+8.3f}  "
                  f"{pf:11.2f}  {mm:+18.1f}  "
                  f"[{sh[0]:+.3f},{sh[1]:+.3f}]  "
                  f"[{cb[0]:+.3f},{cb[1]:+.3f}]  {bl}")
    if args.video and frames:
        import imageio
        imageio.mimsave(args.video, frames, fps=int(round(1 / env.dt)))
        print("video ->", args.video)
    env.close()


if __name__ == "__main__":
    main()
