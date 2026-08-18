"""quad_walk — tip back and walk on FOUR legs (rear-quad animal walk).

The robot rears up nose-high like a begging dog (fronts L0/L5 tucked in
the air), then walks on the four rear legs with a lateral-sequence
animal walk: footfalls LH, LF, RH, RF at quarter-cycle offsets (the
slow-walk order every quadruped uses), duty 0.8, body advancing at
constant velocity with a lateral sway phased against the swinging side.

Level-body four-leg walking is geometrically infeasible on this robot
(rl_move/sim/probe_quad_crawl.py, 08-13: CoM 3.3–5.9 cm ahead of the
best 4-foot polygon). REARING moves the CoM aft over the polygon —
probe_quad_rear.py (08-18) measured 40–74 mm static margin and this
exact gait walking +155 mm / 35 s on the MuJoCo twin with the support
margin never below +34 mm, peak est. current 2.0 A, zero rocking.

Timeline of one demo run (demo time; live speed scales it):
  entry  6.5 s  shift body back → tuck fronts → pitch nose up 17°
  walk          continuous animal walk (fills the middle)
  exit   5.4 s  regather → pitch level → untuck fronts → plant

Everything is world-frame foot anchors + full 3D body-frame IK (the
planted feet truly stay put, including under body pitch — this is what
PlantedBodyIK cannot do, hence a separate module). Pure math, no numpy;
imports only tripod_gait for the leg constants + planar IK.
"""
from __future__ import annotations

import math

FRONT_LEGS = (0, 5)
SUPPORT_LEGS = (1, 2, 3, 4)

# Tucked front "claw" (yaw, hip, knee deg) — quadruped_feasibility
# FRONT_POSES["tuck"], static sweep GO (c57).
TUCK_DEG = (0.0, -63.0, 137.5)

# Sim-tuned gait constants (probe_quad_rear walk, 08-18). Calm regime:
# tilt stays within ~1 deg of the commanded pitch; stride >= 50 mm or
# sway phase far off 125 deg starts slip-rocking — retune in sim first.
PITCH_RAD = math.radians(-17.0)   # rot_y convention: NEGATIVE = nose up
BODY_DX_M = -0.040                # body shift aft in the reared stance
MID_SPLAY_RAD = math.radians(25.0)   # mid feet forward → wider polygon
STRIDE_M = 0.045
LIFT_M = 0.020
PERIOD_S = 3.2                    # s per full 4-step cycle
DUTY = 0.8                        # stance fraction (one leg up at a time)
SWAY_M = 0.025
SWAY_PHASE_RAD = math.radians(125.0)
WALK_PHASE = {2: 0.0, 1: 0.25, 3: 0.5, 4: 0.75}   # LH LF RH RF

# Gait presets. Keys: stride/lift m, period s, duty (stance fraction),
# sway m + phase rad, phase = per-leg footfall offsets (fraction of the
# cycle; supports are L1=LF, L4=RF, L2=LH, L3=RH in the reared quad).
# "walk"  = the sim-tuned lateral-sequence animal walk (one foot up).
# "trot"  = horse trot: DIAGONAL pairs (LF+RH, then RF+LH) — only two
#           feet down mid-beat. Tuned 08-18 under the hardware-speed
#           servo profile (write ~1500 counts/s, verify_noslip's
#           convention — the default fitted 350 profile is 8x slower
#           than what the demo streamer actually writes and turns every
#           gait into a shuffle in sim).
#           The key to real foot CLEARANCE is the deep rear lean
#           (body_dx -80 mm, pitch -20): with the stock -40/-17 stance,
#           lifting a front tips the body about the stance diagonal
#           exactly as fast as the foot rises — the foot SLIDES along
#           the floor (measured: 78% of mid-swing in contact, pitch
#           -16 -> -1 deg). body_dx -90 tips over.
#           On top of the lean, the CALM cadence (period 3.2 s, duty
#           0.68 = real 4-foot overlap between beats, roll 5, sway 22)
#           dominates every faster/wilder variant in the 08-18 DR
#           sweep: 20 mm/s realized (~3x the walk), ~11 deg tilt band,
#           40 mm front apex, and it survives friction x0.6..x1.4,
#           100 ms command latency, and the WHOLE 0.5-2x live-speed
#           range without falling (the old period-2.4/roll-8 preset
#           rocked 56 deg on grippy floors and FELL at 1.25x).
#           speed_cap 1.5 is hardware prudence, not a sim limit —
#           enforced by the demo runner and quad_play.
GAITS: dict[str, dict] = {
    "walk": dict(stride=STRIDE_M, lift=LIFT_M, period=PERIOD_S, duty=DUTY,
                 sway=SWAY_M, sway_phase=SWAY_PHASE_RAD, phase=WALK_PHASE),
    "trot": dict(stride=0.080, lift=0.028, lift_front=0.045,
                 period=3.2, duty=0.68,
                 body_dx=-0.080, pitch=math.radians(-20.0),
                 sway=0.022, sway_phase=math.radians(180.0),
                 roll=math.radians(5.0), roll_phase=math.radians(270.0),
                 phase={1: 0.0, 3: 0.0, 4: 0.5, 2: 0.5},
                 speed_cap=1.5),
}

# entry: shift back · step mids out to the splayed stance (one at a
# time, 5 feet down — scrubbing 25 deg of loaded mid yaw was the v1
# blemish) · tuck fronts · rear up
ENTRY_S = (2.0, 1.2, 2.0, 2.5)
# exit: regather · re-step mids to plant footprint (still reared —
# that's where the rear margin is) · pitch level (body stays aft) ·
# untuck fronts onto their plant offsets · shift forward
EXIT_S = (1.2, 1.2, 2.0, 1.8, 1.3)
ENTRY_TOTAL_S = sum(ENTRY_S)
EXIT_TOTAL_S = sum(EXIT_S)
MIN_SECONDS = ENTRY_TOTAL_S + EXIT_TOTAL_S + PERIOD_S   # ≈ 17 s

# Hardware joint limits (deg) per axis — rl_move.safety AXIS_LIMITS_DEG.
LIMITS_DEG = ((-35.0, 35.0), (-80.0, 30.0), (-20.0, 150.0))


def _smooth(u: float) -> float:
    """cosine ease 0..1"""
    u = min(1.0, max(0.0, u))
    return 0.5 - 0.5 * math.cos(math.pi * u)


class QuadRearWalk:
    """Pure pose function t → 18 joint degrees for the tip-back walk."""

    def __init__(self, base_deg: list[float], seconds: float,
                 gait: str = "walk"):
        import tripod_gait as TG
        self._TG = TG
        self.base = list(base_deg)
        self.seconds = float(seconds)
        g = GAITS[gait]
        # Stance geometry (per-gait): deeper aft shift / pitch moves the
        # CoM back so the stance diagonal can carry the nose during a
        # front swing (otherwise the body tips and the foot slides).
        self.body_dx = g.get("body_dx", BODY_DX_M)
        self.pitch = g.get("pitch", PITCH_RAD)
        self.stride = g["stride"]
        self.lift = g["lift"]
        # Front pair (the splayed mids) may lift higher than the rears —
        # horse-like knee action; purely visual, slip is at the rears.
        self.lift_front = g.get("lift_front", g["lift"])
        self.period = g["period"]
        self.duty = g["duty"]
        self.sway = g["sway"]
        self.sway_phase = g["sway_phase"]
        self.phase = dict(g["phase"])
        # Counter-roll (rad, about x, once per cycle): diagonal 2-foot
        # support has no roll stiffness, so the body droops toward the
        # swinging FRONT leg and eats the foot clearance — command the
        # opposite roll, phased with the beat (horse weight transfer).
        self.roll = g.get("roll", 0.0)
        self.roll_phase = g.get("roll_phase", 0.0)
        # Stance-push (m): when one MID (front) leg swings, its load
        # dumps onto the other mid, whose servo springs compress and the
        # nose sinks exactly as fast as the foot rises (measured: pitch
        # -16 -> -1 deg, foot glued to the floor at 50 mm commanded
        # lift). Pressing the stance mid DEEPER by push*sin(pi*u_swing)
        # holds the nose up — the animal's stance-leg extension.
        self.push = g.get("push", 0.0)
        # Walk window: exit starts EXIT_TOTAL before the end, but never
        # before the entry finishes (too-short runs just rear up + back).
        self.t_exit = max(ENTRY_TOTAL_S, self.seconds - EXIT_TOTAL_S)

        # Foot anchors from the base (plant) pose — yaw-plane frame,
        # z negative below the yaw plane (PlantedBodyIK convention).
        self.anchors: list[list[float]] = []
        self.origins: list[tuple[float, float]] = []
        self.azim: list[float] = []
        for leg in range(6):
            a = (leg + 0.5) * math.pi / 3.0
            yaw = math.radians(self.base[3 * leg + 0])
            hip = math.radians(self.base[3 * leg + 1])
            knee = math.radians(self.base[3 * leg + 2])
            reach = (TG.COXA + TG.FEMUR * math.cos(hip)
                     + TG.TIBIA * math.cos(hip + knee))
            fz = -TG.FEMUR * math.sin(hip) - TG.TIBIA * math.sin(hip + knee)
            ox0 = TG.LEG_RADIAL * math.cos(a)
            oy0 = TG.LEG_RADIAL * math.sin(a)
            self.anchors.append([ox0 + reach * math.cos(a + yaw),
                                 oy0 + reach * math.sin(a + yaw), fz])
            self.origins.append((ox0, oy0))
            self.azim.append(a)
        # Un-splayed plant footprint (exit target); anchors get splayed.
        self.plant_anchors = [list(a) for a in self.anchors]
        # Splay the mid feet forward (+x) about their yaw origins.
        for leg in (1, 4):
            ox0, oy0 = self.origins[leg]
            rx = self.anchors[leg][0] - ox0
            ry = self.anchors[leg][1] - oy0
            sgn = -1.0 if leg == 1 else 1.0
            c = math.cos(sgn * MID_SPLAY_RAD)
            s = math.sin(sgn * MID_SPLAY_RAD)
            self.anchors[leg][0] = ox0 + c * rx - s * ry
            self.anchors[leg][1] = oy0 + s * rx + c * ry

    # -- IK ----------------------------------------------------------------

    def _leg_deg(self, leg: int, wx: float, wy: float, wz: float,
                 bx: float, by: float, pitch: float,
                 roll: float = 0.0) -> tuple[float, ...]:
        """World anchor → (yaw, hip, knee) deg for a body at (bx, by)
        pitched ``pitch`` about y then rolled ``roll`` about x
        (R = rot_y·rot_x; R.T maps world → body)."""
        dx, dy, dz = wx - bx, wy - by, wz
        c, s = math.cos(pitch), math.sin(pitch)
        px = c * dx - s * dz
        py = dy
        pz = s * dx + c * dz
        if roll:
            cr, sr = math.cos(roll), math.sin(roll)
            py, pz = cr * py + sr * pz, -sr * py + cr * pz
        ox0, oy0 = self.origins[leg]
        rx, ry = px - ox0, py - oy0
        a = self.azim[leg]
        ca, sa = math.cos(a), math.sin(a)
        x_yaw = ca * rx + sa * ry
        y_yaw = -sa * rx + ca * ry
        ik = self._TG._leg_ik((math.hypot(x_yaw, y_yaw), 0.0, pz))
        if ik is None:      # out of envelope: hold the base angles
            return tuple(self.base[3 * leg: 3 * leg + 3])
        hip, knee = ik
        out = (math.degrees(math.atan2(y_yaw, x_yaw)),
               math.degrees(hip), math.degrees(knee))
        return tuple(max(lo, min(hi, v))
                     for v, (lo, hi) in zip(out, LIMITS_DEG))

    def _solve(self, bx: float, by: float, pitch: float,
               feet: dict[int, tuple[float, float, float]],
               front_deg, roll: float = 0.0) -> list[float]:
        pose = list(self.base)
        for leg in FRONT_LEGS:
            pose[3 * leg: 3 * leg + 3] = front_deg[
                3 * leg: 3 * leg + 3] if len(front_deg) == 18 else front_deg
        for leg in SUPPORT_LEGS:
            wx, wy, wz = feet[leg]
            pose[3 * leg: 3 * leg + 3] = self._leg_deg(
                leg, wx, wy, wz, bx, by, pitch, roll)
        return pose

    # -- walk-phase foot schedule -------------------------------------------

    def _walk_feet(self, tw: float, *, freeze_swing: bool = False
                   ) -> dict[int, tuple[float, float, float]]:
        """Support-foot world targets ``tw`` seconds into the walk.
        ``freeze_swing`` plants a mid-swing leg at its landing anchor
        (used to build the regather target of the exit)."""
        feet = {}
        swing_u = {}
        for leg in SUPPORT_LEGS:
            ph = tw / self.period - self.phase[leg]
            n = math.floor(ph)
            sph = ph - n
            ax, ay, az = self.anchors[leg]
            # center each leg's stance sweep in its workspace
            a0x = ax - self.stride * (
                1.0 - self.phase[leg] - (2 - self.duty) / 2)
            if sph < (1.0 - self.duty) and not freeze_swing:
                u = sph / (1.0 - self.duty)
                swing_u[leg] = u
                lift = self.lift_front if leg in (1, 4) else self.lift
                feet[leg] = (a0x + (n + _smooth(u)) * self.stride, ay,
                             az + lift * math.sin(math.pi * u))
            else:
                feet[leg] = (a0x + (n + 1) * self.stride, ay, az)
        if self.push and not freeze_swing:
            # stance mid presses down while the other mid swings
            for leg, other in ((1, 4), (4, 1)):
                if leg not in swing_u and other in swing_u:
                    fx, fy, fz = feet[leg]
                    feet[leg] = (fx, fy, fz - self.push * math.sin(
                        math.pi * swing_u[other]))
        return feet

    def _walk_body(self, tw: float) -> tuple[float, float]:
        v = self.stride / self.period
        sway = self.sway * math.sin(2 * math.pi * tw / self.period
                                    + self.sway_phase)
        return self.body_dx + v * tw, sway

    def _walk_roll(self, tw: float) -> float:
        if not self.roll:
            return 0.0
        return self.roll * math.sin(2 * math.pi * tw / self.period
                                    + self.roll_phase)

    # -- the pose function ---------------------------------------------------

    def pose_at(self, t: float) -> list[float]:
        t = max(0.0, float(t))
        tuckq = list(self.base)
        for leg in FRONT_LEGS:
            tuckq[3 * leg: 3 * leg + 3] = TUCK_DEG
        plant_feet = {leg: tuple(self.anchors[leg])
                      for leg in SUPPORT_LEGS}

        e1, e1b, e2, e3 = ENTRY_S
        plant_home = {leg: tuple(self.plant_anchors[leg])
                      for leg in SUPPORT_LEGS}
        if t < e1:                       # entry 1: shift back, 6 planted
            u = _smooth(t / e1)
            bx = u * self.body_dx
            pose = self._solve(bx, 0.0, 0.0, plant_home, self.base)
            for leg in FRONT_LEGS:       # fronts still planted
                wx, wy, wz = self.anchors[leg]
                pose[3 * leg: 3 * leg + 3] = self._leg_deg(
                    leg, wx, wy, wz, bx, 0.0, 0.0)
            return pose
        if t < e1 + e1b:                 # entry 1b: mids step out to splay
            prog = (t - e1) / e1b        # (L1 then L4; 5 feet stay down)
            feet = dict(plant_home)
            for j, leg in enumerate((1, 4)):
                u = _smooth(min(1.0, max(0.0, 2.0 * prog - j)))
                px, py, pz = self.plant_anchors[leg]
                sx, sy, sz = self.anchors[leg]
                lift = LIFT_M * math.sin(math.pi * min(
                    1.0, max(0.0, 2.0 * prog - j)))
                feet[leg] = (px + u * (sx - px), py + u * (sy - py),
                             pz + lift)
            pose = self._solve(self.body_dx, 0.0, 0.0, feet, self.base)
            for leg in FRONT_LEGS:
                wx, wy, wz = self.anchors[leg]
                pose[3 * leg: 3 * leg + 3] = self._leg_deg(
                    leg, wx, wy, wz, self.body_dx, 0.0, 0.0)
            return pose
        if t < e1 + e1b + e2:            # entry 2: tuck the fronts
            u = _smooth((t - e1 - e1b) / e2)
            fq = [a + u * (b - a) for a, b in zip(self.base, tuckq)]
            return self._solve(self.body_dx, 0.0, 0.0, plant_feet, fq)
        if t < ENTRY_TOTAL_S:            # entry 3: rear up
            u = _smooth((t - e1 - e1b - e2) / e3)
            return self._solve(self.body_dx, 0.0, u * self.pitch,
                               plant_feet, tuckq)

        tw_end = self.t_exit - ENTRY_TOTAL_S    # walk window length
        if t < self.t_exit:              # ---- the animal walk ----
            tw = t - ENTRY_TOTAL_S
            bx, by = self._walk_body(tw)
            return self._solve(bx, by, self.pitch, self._walk_feet(tw),
                               tuckq, roll=self._walk_roll(tw))

        # ---- exit (reverse of the entry, ending at the plant pose) ----
        x1, x2, x3, x4, x5 = EXIT_S
        tx = t - self.t_exit
        bx_end = self.body_dx + (self.stride / self.period) * tw_end
        feet_end = self._walk_feet(tw_end, freeze_swing=True)
        # Final body center xf: where the REAR feet already sit at their
        # plant offsets (no rear re-step; residual is < stride/4).
        xf = 0.5 * sum(feet_end[r][0] - self.plant_anchors[r][0]
                       for r in (2, 3))
        bx_aft = xf + self.body_dx

        if tx < x1:                      # 1: regather to all-4-planted
            u = _smooth(tx / x1)
            q_end = self._solve(*self._walk_body(tw_end), self.pitch,
                                self._walk_feet(tw_end), tuckq,
                                roll=self._walk_roll(tw_end))
            q_gather = self._solve(bx_end, 0.0, self.pitch, feet_end,
                                   tuckq)
            return [a + u * (b - a) for a, b in zip(q_end, q_gather)]

        # mid feet re-step to the (un-splayed) plant footprint at xf,
        # one at a time, still reared — the walk swings proved this.
        def _mid_feet(prog: float) -> dict:
            feet = dict(feet_end)
            for j, leg in enumerate((1, 4)):
                u = _smooth(min(1.0, max(0.0, 2.0 * prog - j)))
                fx0, fy0, fz0 = feet_end[leg]
                px, py, pz = self.plant_anchors[leg]
                lift = LIFT_M * math.sin(math.pi * min(
                    1.0, max(0.0, 2.0 * prog - j)))
                feet[leg] = (fx0 + u * (px + xf - fx0),
                             fy0 + u * (py - fy0), pz + lift)
            return feet

        if tx < x1 + x2:                 # 2: mids step home (L1 then L4)
            prog = (tx - x1) / x2
            return self._solve(bx_end, 0.0, self.pitch,
                               _mid_feet(prog), tuckq)
        feet_home = _mid_feet(1.0)
        # body glides bx_end -> bx_aft across phases 2-3 (tiny at 40 s)
        if tx < x1 + x2 + x3:            # 3: pitch level, body stays aft
            u = _smooth((tx - x1 - x2) / x3)
            bx = bx_end + u * (bx_aft - bx_end)
            return self._solve(bx, 0.0, (1.0 - u) * self.pitch,
                               feet_home, tuckq)
        # front targets: plant offsets relative to the FINAL center xf
        fq_target = list(self.base)
        for leg in FRONT_LEGS:
            px, py, pz = self.plant_anchors[leg]
            fq_target[3 * leg: 3 * leg + 3] = self._leg_deg(
                leg, px + xf, py, pz, bx_aft, 0.0, 0.0)
        if tx < x1 + x2 + x3 + x4:       # 4: untuck fronts onto plant spots
            u = _smooth((tx - x1 - x2 - x3) / x4)
            fq = [a + u * (b - a) for a, b in zip(tuckq, fq_target)]
            return self._solve(bx_aft, 0.0, 0.0, feet_home, fq)
        # 5: shift forward to xf with all 6 planted -> exact plant pose
        u = _smooth((tx - x1 - x2 - x3 - x4) / x5)
        bx = bx_aft + u * (xf - bx_aft)
        pose = self._solve(bx, 0.0, 0.0, feet_home, self.base)
        for leg in FRONT_LEGS:
            px, py, pz = self.plant_anchors[leg]
            pose[3 * leg: 3 * leg + 3] = self._leg_deg(
                leg, px + xf, py, pz, bx, 0.0, 0.0)
        return pose


def make_quad_walk_pose_fn(base_deg: list[float], seconds: float,
                           gait: str = "walk"):
    """Duration-aware factory: the exit is scripted into the last
    ~7.5 s of ``seconds`` so the run ends back at the plant pose."""
    return QuadRearWalk(base_deg, seconds, gait=gait).pose_at
