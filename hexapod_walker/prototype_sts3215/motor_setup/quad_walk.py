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

Original full-demo timeline (demo time; live speed scales it):
  entry  6.5 s  shift body back → tuck fronts → pitch nose up 17°
  walk          continuous animal walk (fills the middle)
  exit   5.4 s  regather → pitch level → untuck fronts → plant

The web UI also uses the same generator as separate phases: rear up and
hold, walk-only forward/backward while reared, then come down on request.

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
# FRONT_POSES["tuck"], static sweep GO (c57).  Knee was 137.5 but the
# real servo hard-stops at ~120 (08-18 telemetry: cmd clamped to 128,
# present pinned at 119.9, 2.2 A grind for the whole run) — stay under.
TUCK_DEG = (0.0, -63.0, 112.0)

# Sim-tuned gait constants (probe_quad_rear walk, 08-18). Calm regime:
# tilt stays within ~1 deg of the commanded pitch; stride >= 50 mm or
# sway phase far off 125 deg starts slip-rocking — retune in sim first.
PITCH_RAD = math.radians(-17.0)   # rot_y convention: NEGATIVE = nose up
BODY_DX_M = -0.040                # body shift aft in the reared stance
BODY_Z_M = 0.020                  # extra body clearance while reared
MID_SPLAY_RAD = math.radians(8.0)    # compact mid-foot splay; 25 deg braced
                                     # the real robot too wide to rear back
STRIDE_M = 0.045
LIFT_M = 0.020
PERIOD_S = 3.2                    # s per full 4-step cycle
DUTY = 0.8                        # stance fraction (one leg up at a time)
SWAY_M = 0.025
SWAY_PHASE_RAD = math.radians(125.0)
WALK_PHASE = {2: 0.0, 1: 0.25, 3: 0.5, 4: 0.75}   # LH LF RH RF

# Gait presets. Keys: stride/lift/body_z m, period s, duty (stance
# fraction), sway m + phase rad, phase = per-leg footfall offsets
# (fraction of the cycle; supports are L1=LF, L4=RF, L2=LH, L3=RH in
# the reared quad).
# "walk"  = the sim-tuned lateral-sequence animal walk (one foot up).
# "trot"  = horse trot: DIAGONAL pairs (LF+RH, then RF+LH) — only two
#           feet down mid-beat. Tuned 08-18 under a ~1500 counts/s
#           servo profile.  IMPORTANT (08-18 hardware debug): every
#           WritePosEx restarts the servo trapezoid from zero velocity,
#           so the realized speed ceiling is set by acc x write period,
#           NOT the commanded speed — at the old 20 Hz stream this was
#           ~500 counts/s and the trot pranced in place.  The demo
#           streamer now writes quad gaits at 10 Hz / acc 254
#           (QUAD_STREAM_TICK_S), which realizes ~14 mm/s with 25-33 mm
#           front apex in the restart-accurate sim.
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
#           Hardware note 08-20: the real robot was still falling forward
#           on walk/trot, so the deployed presets below are deliberately
#           more conservative than the 08-18 sim showcase: more nose-up
#           pitch without pushing the mid-yaw joints into stops, shorter
#           steps, slower cadence, and a hard trot cap.
GAITS: dict[str, dict] = {
    # 08-18 hardware debug: the ORIGINAL shallow-stance walk (-40 mm /
    # -17 deg) marched perfectly in place on the real floor — lifting a
    # front foot just tipped the body onto it (restart-sim apex 2-5 mm
    # no matter the commanded lift).  The trot's DEEP aft lean fixes
    # the same mechanism for the walk. Hardware now gets more body pitch
    # (-24 deg) with less rearward translation (-70 mm), which keeps the
    # mid-leg yaw joints off their hard stops better than -80/-20 did.
    "walk": dict(stride=0.035, lift=0.026, lift_front=0.035,
                 period=5.0, duty=0.82,
                 body_dx=-0.070, pitch=math.radians(-24.0),
                 body_z=0.020,
                 splay=math.radians(8.0),
                 sway=0.020, sway_phase=SWAY_PHASE_RAD, phase=WALK_PHASE),
    "walk_pitch": dict(stride=0.030, lift=0.024, lift_front=0.032,
                       period=5.4, duty=0.84,
                       body_dx=-0.060, pitch=math.radians(-28.0),
                       body_z=0.026,
                       splay=math.radians(6.0),
                       sway=0.018, sway_phase=SWAY_PHASE_RAD,
                       phase=WALK_PHASE, speed_cap=0.8),
    "walk_aggressive": dict(stride=0.025, lift=0.022, lift_front=0.028,
                            period=6.0, duty=0.86,
                            body_dx=-0.050, pitch=math.radians(-32.0),
                            body_z=0.032,
                            splay=math.radians(4.0),
                            sway=0.014, sway_phase=SWAY_PHASE_RAD,
                            phase=WALK_PHASE, speed_cap=0.6),
    "trot": dict(stride=0.050, lift=0.022, lift_front=0.032,
                 period=4.8, duty=0.76,
                 body_dx=-0.070, pitch=math.radians(-24.0),
                 body_z=0.020,
                 splay=math.radians(8.0),
                 sway=0.016, sway_phase=math.radians(180.0),
                 roll=math.radians(3.0), roll_phase=math.radians(270.0),
                 phase={1: 0.0, 3: 0.0, 4: 0.5, 2: 0.5},
                 speed_cap=0.7),
    "trot_pitch": dict(stride=0.040, lift=0.020, lift_front=0.030,
                       period=5.2, duty=0.80,
                       body_dx=-0.060, pitch=math.radians(-28.0),
                       body_z=0.026,
                       splay=math.radians(6.0),
                       sway=0.014, sway_phase=math.radians(180.0),
                       roll=math.radians(2.5),
                       roll_phase=math.radians(270.0),
                       phase={1: 0.0, 3: 0.0, 4: 0.5, 2: 0.5},
                       speed_cap=0.6),
    "trot_aggressive": dict(stride=0.032, lift=0.018, lift_front=0.024,
                            period=5.8, duty=0.82,
                            body_dx=-0.050, pitch=math.radians(-32.0),
                            body_z=0.032,
                            splay=math.radians(4.0),
                            sway=0.012,
                            sway_phase=math.radians(180.0),
                            roll=math.radians(2.0),
                            roll_phase=math.radians(270.0),
                            phase={1: 0.0, 3: 0.0, 4: 0.5, 2: 0.5},
                            speed_cap=0.5),
    # "rear" = HOLD the reared stance without stepping: stride/lift 0
    # turns the walk-phase math into a no-op (all four support feet stay
    # planted) while a gentle sway keeps the pose alive.  The -24 deg
    # nose-up hold gives the walking phases a safer launch pose.
    # Used by the dance's stallion act, which overlays front-paw
    # gestures on the hold window (inplace_demos._make_stallion_fn).
    "rear": dict(stride=0.0, lift=0.0, lift_front=0.0,
                 period=4.0, duty=0.75,
                 body_dx=-0.070, pitch=math.radians(-24.0),
                 body_z=0.020,
                 splay=math.radians(8.0),
                 sway=0.015, sway_phase=SWAY_PHASE_RAD, phase=WALK_PHASE,
                 speed_cap=1.5),
    "rear_pitch": dict(stride=0.0, lift=0.0, lift_front=0.0,
                       period=4.0, duty=0.75,
                       body_dx=-0.060, pitch=math.radians(-28.0),
                       body_z=0.026,
                       splay=math.radians(6.0),
                       sway=0.012, sway_phase=SWAY_PHASE_RAD,
                       phase=WALK_PHASE, speed_cap=1.0),
    "rear_aggressive": dict(stride=0.0, lift=0.0, lift_front=0.0,
                            period=4.0, duty=0.75,
                            body_dx=-0.050, pitch=math.radians(-32.0),
                            body_z=0.032,
                            splay=math.radians(4.0),
                            sway=0.010, sway_phase=SWAY_PHASE_RAD,
                            phase=WALK_PHASE, speed_cap=0.8),
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
                 gait: str = "walk", direction: float = 1.0,
                 trim_fn=None):
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
        self.body_z = float(g.get("body_z", BODY_Z_M))
        self.stride = g["stride"] * (-1.0 if float(direction) < 0.0 else 1.0)
        self.lift = g["lift"]
        # Front pair (the splayed mids) may lift higher than the rears —
        # horse-like knee action; purely visual, slip is at the rears.
        self.lift_front = g.get("lift_front", g["lift"])
        self.period = g["period"]
        self.duty = g["duty"]
        self.sway = g["sway"]
        self.sway_phase = g["sway_phase"]
        self.phase = dict(g["phase"])
        self.splay = float(g.get("splay", MID_SPLAY_RAD))
        self.trim_fn = trim_fn
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
            reach, fz = TG.foot_rz_from_hip_knee(
                math.degrees(hip), math.degrees(knee))
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
            c = math.cos(sgn * self.splay)
            s = math.sin(sgn * self.splay)
            self.anchors[leg][0] = ox0 + c * rx - s * ry
            self.anchors[leg][1] = oy0 + s * rx + c * ry

    # -- IK ----------------------------------------------------------------

    def _leg_deg(self, leg: int, wx: float, wy: float, wz: float,
                 bx: float, by: float, pitch: float,
                 roll: float = 0.0, bz: float = 0.0) -> tuple[float, ...]:
        """World anchor → (yaw, hip, knee) deg for a body at (bx, by)
        and height ``bz`` above the plant frame, pitched ``pitch`` about
        y then rolled ``roll`` about x (R = rot_y·rot_x; R.T maps world
        → body)."""
        dx, dy, dz = wx - bx, wy - by, wz - bz
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
               front_deg, roll: float = 0.0, bz: float = 0.0) -> list[float]:
        pose = list(self.base)
        for leg in FRONT_LEGS:
            pose[3 * leg: 3 * leg + 3] = front_deg[
                3 * leg: 3 * leg + 3] if len(front_deg) == 18 else front_deg
        for leg in SUPPORT_LEGS:
            wx, wy, wz = feet[leg]
            pose[3 * leg: 3 * leg + 3] = self._leg_deg(
                leg, wx, wy, wz, bx, by, pitch, roll, bz)
        return pose

    def _trim(self) -> tuple[float, float]:
        """Live balance trim as (body_dx_m, pitch_rad)."""
        if not callable(self.trim_fn):
            return 0.0, 0.0
        try:
            tr = self.trim_fn()
        except Exception:
            return 0.0, 0.0
        if not tr:
            return 0.0, 0.0
        try:
            if isinstance(tr, dict):
                dx = float(tr.get("body_dx_m", 0.0))
                pitch = float(tr.get("pitch_rad", 0.0))
            else:
                dx, pitch = tr
                dx = float(dx)
                pitch = float(pitch)
        except (TypeError, ValueError):
            return 0.0, 0.0
        # Second-line safety clamp; the IMU controller also limits these.
        dx = max(-0.018, min(0.018, dx))
        pitch = max(math.radians(-7.0), min(math.radians(7.0), pitch))
        return dx, pitch

    def _trim_body_pitch(self, bx: float, pitch: float
                         ) -> tuple[float, float]:
        dx, dp = self._trim()
        return bx + dx, pitch + dp

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

    def walk_all_stance_at(self, tw: float) -> bool:
        """True when no support foot is in its swing window at walk-clock
        ``tw`` — the only safe moment to freeze the gait (balance
        recovery brace-hold): freezing mid-swing strands a foot in the
        air on tripod support, which measurably deepens a tip."""
        if self.stride == 0.0 and self.lift == 0.0 and self.lift_front == 0.0:
            return True         # hold gaits: all four always planted
        p = (tw / self.period) % 1.0
        return all(((p - ph) % 1.0) >= (1.0 - self.duty)
                   for ph in self.phase.values())

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

    def _tucked_fronts(self) -> list[float]:
        tuckq = list(self.base)
        for leg in FRONT_LEGS:
            tuckq[3 * leg: 3 * leg + 3] = TUCK_DEG
        return tuckq

    def _support_feet(self) -> dict[int, tuple[float, float, float]]:
        return {leg: tuple(self.anchors[leg]) for leg in SUPPORT_LEGS}

    def reared_pose(self) -> list[float]:
        bx, pitch = self._trim_body_pitch(self.body_dx, self.pitch)
        return self._solve(bx, 0.0, pitch,
                           self._support_feet(), self._tucked_fronts(),
                           bz=self.body_z)

    def reared_hold_pose_at(self, _t: float) -> list[float]:
        """Static all-four-support reared hold.

        Used by the Quad tab's Stop/Hold action. It intentionally has no
        entry timeline: when already reared, the safest stop target is the
        known stable support pose, not an arbitrary frozen gait frame.
        """
        return self.reared_pose()

    def entry_hold_pose_at(self, t: float) -> list[float]:
        """Entry choreography, then hold the reared stance indefinitely."""
        t = max(0.0, float(t))
        if t < ENTRY_TOTAL_S:
            return self.pose_at(t)
        return self.walk_only_pose_at(t - ENTRY_TOTAL_S)

    def walk_only_pose_at(self, t: float) -> list[float]:
        """Walk while already reared; no entry or automatic exit."""
        tw = max(0.0, float(t))
        bx, by = self._walk_body(tw)
        bx, pitch = self._trim_body_pitch(bx, self.pitch)
        return self._solve(bx, by, pitch, self._walk_feet(tw),
                           self._tucked_fronts(), roll=self._walk_roll(tw),
                           bz=self.body_z)

    def exit_pose_at(self, t: float) -> list[float]:
        """Come down from a stable reared hold back to the plant stance."""
        return self._exit_pose_at(max(0.0, float(t)), 0.0)

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
            bz = 0.35 * u * self.body_z
            pose = self._solve(bx, 0.0, 0.0, plant_home, self.base, bz=bz)
            for leg in FRONT_LEGS:       # fronts still planted
                wx, wy, wz = self.anchors[leg]
                pose[3 * leg: 3 * leg + 3] = self._leg_deg(
                    leg, wx, wy, wz, bx, 0.0, 0.0, bz=bz)
            return pose
        if t < e1 + e1b:                 # entry 1b: mids step out to splay
            prog = (t - e1) / e1b        # (L1 then L4; 5 feet stay down)
            bz = 0.35 * self.body_z
            feet = dict(plant_home)
            for j, leg in enumerate((1, 4)):
                u = _smooth(min(1.0, max(0.0, 2.0 * prog - j)))
                px, py, pz = self.plant_anchors[leg]
                sx, sy, sz = self.anchors[leg]
                lift = LIFT_M * math.sin(math.pi * min(
                    1.0, max(0.0, 2.0 * prog - j)))
                feet[leg] = (px + u * (sx - px), py + u * (sy - py),
                             pz + lift)
            pose = self._solve(self.body_dx, 0.0, 0.0, feet, self.base, bz=bz)
            for leg in FRONT_LEGS:
                wx, wy, wz = self.anchors[leg]
                pose[3 * leg: 3 * leg + 3] = self._leg_deg(
                    leg, wx, wy, wz, self.body_dx, 0.0, 0.0, bz=bz)
            return pose
        if t < e1 + e1b + e2:            # entry 2: tuck the fronts
            u = _smooth((t - e1 - e1b) / e2)
            fq = [a + u * (b - a) for a, b in zip(self.base, tuckq)]
            return self._solve(self.body_dx, 0.0, 0.0, plant_feet, fq,
                               bz=0.35 * self.body_z)
        if t < ENTRY_TOTAL_S:            # entry 3: rear up
            u = _smooth((t - e1 - e1b - e2) / e3)
            bz = (0.35 + 0.65 * u) * self.body_z
            return self._solve(self.body_dx, 0.0, u * self.pitch,
                               plant_feet, tuckq, bz=bz)

        tw_end = self.t_exit - ENTRY_TOTAL_S    # walk window length
        if t < self.t_exit:              # ---- the animal walk ----
            tw = t - ENTRY_TOTAL_S
            bx, by = self._walk_body(tw)
            bx, pitch = self._trim_body_pitch(bx, self.pitch)
            return self._solve(bx, by, pitch, self._walk_feet(tw),
                               tuckq, roll=self._walk_roll(tw),
                               bz=self.body_z)

        return self._exit_pose_at(t - self.t_exit, tw_end)

    def _exit_pose_at(self, tx: float, tw_end: float) -> list[float]:
        """Exit choreography ending at the plant pose.

        ``tw_end`` is the walk-time whose support-foot layout should be
        regathered first. For the standalone come-down command this is 0:
        a stable all-four-support reared stance.
        """
        tx = max(0.0, float(tx))
        tuckq = self._tucked_fronts()
        x1, x2, x3, x4, x5 = EXIT_S
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
                                roll=self._walk_roll(tw_end),
                                bz=self.body_z)
            q_gather = self._solve(bx_end, 0.0, self.pitch, feet_end,
                                   tuckq, bz=self.body_z)
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
                               _mid_feet(prog), tuckq, bz=self.body_z)
        feet_home = _mid_feet(1.0)
        # body glides bx_end -> bx_aft across phases 2-3 (tiny at 40 s)
        if tx < x1 + x2 + x3:            # 3: pitch level, body stays aft
            u = _smooth((tx - x1 - x2) / x3)
            bx = bx_end + u * (bx_aft - bx_end)
            bz = (1.0 - u) * self.body_z
            return self._solve(bx, 0.0, (1.0 - u) * self.pitch,
                               feet_home, tuckq, bz=bz)
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
                           gait: str = "walk", direction: float = 1.0,
                           phase: str = "full", trim_fn=None):
    """Duration-aware factory for full or split quad-mode phases.

    The returned callable carries an ``all_stance_at(t)`` attribute
    (streamer time -> "all four support feet planted") where the phase
    schedule is known; balance recovery uses it to brace-hold only at
    stance-stable moments.  ``None`` where a freeze is never safe.
    """
    q = QuadRearWalk(
        base_deg, seconds, gait=gait, direction=direction, trim_fn=trim_fn)
    phase = (phase or "full").strip().lower()
    if phase in ("rear", "entry", "entry_hold"):
        base_fn = q.entry_hold_pose_at

        def all_stance_at(t: float) -> bool:
            tw = float(t) - ENTRY_TOTAL_S
            return tw >= 0.0 and q.walk_all_stance_at(tw)
    elif phase in ("hold", "hold_only", "settle", "static", "reared_hold"):
        base_fn = q.reared_hold_pose_at

        def all_stance_at(t: float) -> bool:
            return True
    elif phase in ("walk", "drive"):
        base_fn = q.walk_only_pose_at
        all_stance_at = q.walk_all_stance_at
    elif phase in ("down", "exit"):
        # Exit choreography re-steps the mid feet on its own clock;
        # never freeze it.
        base_fn = q.exit_pose_at
        all_stance_at = None
    else:
        base_fn = q.pose_at

        def all_stance_at(t: float) -> bool:
            tw = float(t) - ENTRY_TOTAL_S
            return (0.0 <= tw and float(t) < q.t_exit
                    and q.walk_all_stance_at(tw))

    def pose_fn(t: float) -> list[float]:
        return base_fn(t)

    pose_fn.all_stance_at = all_stance_at
    return pose_fn
