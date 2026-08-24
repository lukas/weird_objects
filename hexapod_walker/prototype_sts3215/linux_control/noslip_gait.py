"""World-pinned tripod gait: a continuum from step-then-shift to
continuous body motion, selected by ``alpha``.

Rules-based, no RL. The structural difference from
``tripod_gait.TripodGait`` (which drags stance feet linearly through the
BODY frame): every planted foot here is pinned to a fixed anchor point
in a dead-reckoned world frame. At ``alpha = 0`` (default, the original
step-then-shift behaviour) the cycle alternates two mutually exclusive
activities:

  SHIFT  all six feet planted; the body translates/rotates between the
         pinned anchors on a cosine ease (zero body velocity at both
         ends of the phase).
  STEP   the body holds still; one tripod swings to its next anchors on
         a min-jerk profile (zero touchdown velocity) with half-sine
         lift. The stance tripod's targets do not move at all.
  DWELL  a short settle after each step with everything commanded
         stationary, so servo tracking lag on the swing legs is
         absorbed while those feet are still unloaded — without it a
         late-landing foot drags across the ground catching up during
         the next shift (measured: that drag dominated slip when the
         swing sweep saturated the servo write profile).

``alpha`` in (0, 1] moves ``alpha``'s share of the cycle twist out of
the eased shift phases and spreads it EVENLY across the whole cycle, so
the body keeps moving during swings and dwells. At ``alpha = 1`` the
shift phases carry no ease pulse at all and the body translates at a
constant rate through the entire cycle — a continuous world-pinned
gait. Crucially the no-slip guarantee is IDENTICAL at every alpha:

  - planted feet are commanded to fixed world anchors, so their
    commanded ground-relative velocity is exactly zero regardless of
    what the body is doing;
  - a swing interpolates between two FIXED world points (min-jerk), so
    the commanded touchdown velocity in the WORLD frame is zero even
    while the body moves — the body-frame touchdown velocity is then
    automatically matched to ground speed (no scuff).

What alpha trades: higher alpha lowers the peak body velocity for the
same commanded speed (the alpha=0 shift phases squeeze all body travel
into ~48% of the cycle), so servo tracking is cleaner at shorter
periods — but the body is in motion while only one tripod supports it,
so the "pause at any instant" quasi-static property below is strictest
at alpha = 0. Residual slip at every alpha is bounded by servo
tracking error only, never by the gait's own kinematics. (The
body-frame drag gait loses ~50% of its stride to slip on hardware; see
``rl_move/sim/calibrate_slip.py``.)

Each foot is planted 80% of the cycle and each swing tripod lifts while
the other tripod (a statically stable triangle) holds the body; at
alpha = 0 the gait is fully quasi-static: it can pause at any instant
without falling.

``groups`` generalizes the swing pattern to the other classic hexapod
gaits — the phase machine becomes K repetitions of (shift, swing
group_k, dwell) per cycle, and the world-anchor no-slip guarantee is
untouched because it never depended on WHICH feet swing together:

  TRIPOD  ((0,2,4), (1,3,5))            3 feet down, fastest
  RIPPLE  ((0,3), (1,4), (2,5))         4 feet down — opposite pairs
  WAVE    ((0,), (3,), (1,), (4,), (2,), (5,))   5 feet down, steadiest
          (single legs, alternating sides so support stays centred)

Presets ``NoSlipGait.ripple()`` / ``NoSlipGait.wave()`` follow the
clamp-fit philosophy (alpha = 1 constant-rate drift, timing sized for
the fitted ~31 deg/s servo cruise clamp).  Leg numbering: 0 front-left,
1 mid-left, 2 rear-left, 3 rear-right, 4 mid-right, 5 front-right.

Stdlib-only (like ``tripod_gait``) so it can be vendored to the Uno Q.
Verify in sim with:  uv run python -m rl_move.sim.verify_noslip
(``--gait ripple`` / ``--gait wave`` replay the presets).
"""
from __future__ import annotations

import math

from tripod_gait import (COXA, FEMUR, TIBIA, LEG_RADIAL, _clip, _leg_ik,
                         _plant_hip_knee_deg, foot_rz_from_hip_knee)

YAW_LIM = math.radians(35.0)
HIP_LIM = (math.radians(-80.0), math.radians(30.0))
KNEE_LIM = (math.radians(-20.0), math.radians(150.0))

# Tangential hip-pitch-axis offset (m): hexapod_prototype.COXA_HIP_ANCHOR_Y
# (-25.65 mm, the documented pinwheel). The femur/tibia plane rides at this
# constant lateral offset in the yaw frame, and the offset's world direction
# ROTATES with the yaw joint — the planar IK that tripod_gait/body_ik use
# ignores it, which smears a foot ~9 mm sideways across a +/-22 deg yaw
# sweep. A no-slip gait must solve the offset-arm yaw exactly.
HIP_Y = -0.02565


def _minjerk(p: float) -> float:
    """Zero velocity AND acceleration at both ends."""
    p = _clip(p, 0.0, 1.0)
    return p * p * p * (10.0 - 15.0 * p + 6.0 * p * p)


def _ease(p: float) -> float:
    """Cosine ease: zero velocity at both ends (body shift profile)."""
    p = _clip(p, 0.0, 1.0)
    return 0.5 - 0.5 * math.cos(math.pi * p)


class NoSlipGait:
    """World-pinned tripod gait; same call surface as ``TripodGait``.

    Phase cycle (fractions of ``period``):
        shift -> swing tripod A (0,2,4) -> dwell
        shift -> swing tripod B (1,3,5) -> dwell

    Commanded velocity is realized as one body-frame twist per cycle
    (vx*T, vy*T, omega*T), clamped to the leg workspace. ``alpha``
    splits that twist between two body-motion components:

      - ``1 - alpha`` is delivered by the two SHIFT phases on a cosine
        ease (the step-then-shift pulse; zero body velocity at both
        phase ends);
      - ``alpha`` is delivered as a constant-rate drift across the
        WHOLE cycle (shift, swing and dwell alike).

    alpha = 0 is the original step-then-shift gait; alpha = 1 is a
    continuous world-pinned gait. Swing targets are the neutral foot
    positions under the body pose HALFWAY (in body-progress terms)
    through the foot's next planted interval, so leg excursions stay
    symmetric about neutral at every alpha. No body motion is commanded
    before the second shift phase of a run, so neither tripod ever sees
    more than about half a stride of excursion at startup.
    """

    MAX_VX = 0.040
    MAX_VY = 0.035
    MAX_OMEGA = 0.30
    STRIDE_MAX = 0.080      # m of body travel per full cycle
    YAW_STEP_MAX = 0.35     # rad of body yaw per full cycle

    # Timing that fits the fitted ~31 deg/s servo cruise clamp (the RL
    # training env's write 400 / acc 20 / slew 1.5 deg-per-tick profile).
    # Measured 2026-08-12 with verify_noslip's contact-Jacobian scrub:
    # zero true scrub, travel ratio 0.96 at 13 mm/s commanded, and the
    # lowest loaded foot-centre drift of the sweep (59 mm/20 s vs 345 mm
    # for the default timing) — the constant-rate alpha=1 drift is what
    # keeps peak joint rates under the clamp.
    CLAMP_FIT_KW = dict(period=6.0, lift=0.020, shift_frac=0.10,
                        swing_frac=0.40, alpha=1.0)

    TRIPOD_A = (0, 2, 4)
    TRIPOD_B = (1, 3, 5)
    GROUPS_TRIPOD = (TRIPOD_A, TRIPOD_B)
    # Opposite pairs: removing any pair leaves a wide 4-foot quad with
    # the CoM well inside — statically stable through every swing.
    GROUPS_RIPPLE = ((0, 3), (1, 4), (2, 5))
    # One leg at a time, alternating sides (each consecutive swing is
    # the OPPOSITE leg of the previous one) so the 5-foot support never
    # leans to one side for long.
    GROUPS_WAVE = ((0,), (3,), (1,), (4,), (2,), (5,))

    # Presets tuned like CLAMP_FIT_KW (alpha=1, sized so per-phase joint
    # rates fit the ~31 deg/s fitted servo cruise clamp; validated with
    # rl_move/sim/verify_noslip.py --gait ripple / wave).  Every swing
    # must recover a full-cycle stride no matter how many groups there
    # are, so more groups need a LONGER period to keep the same ~2.4 s
    # swing the clamp-fit tripod uses — wave is inherently the slowest.
    RIPPLE_KW = dict(period=8.0, lift=0.020, shift_frac=0.02,
                     swing_frac=0.30, alpha=1.0)
    WAVE_KW = dict(period=20.0, lift=0.018, shift_frac=0.008,
                   swing_frac=0.155, alpha=1.0)

    def __init__(
        self,
        *,
        period: float = 3.2,
        lift: float = 0.028,
        shift_frac: float = 0.24,
        swing_frac: float = 0.22,
        vx: float = 0.0,
        vy: float = 0.0,
        omega: float = 0.0,
        alpha: float = 0.0,
        groups: tuple = GROUPS_TRIPOD,
    ):
        self.period = max(float(period), 0.4)
        self.lift = _clip(float(lift), 0.005, 0.05)
        self.alpha = _clip(float(alpha), 0.0, 1.0)
        self.groups = tuple(tuple(int(i) for i in g) for g in groups)
        k = len(self.groups)
        assert k >= 2 and sorted(
            i for g in self.groups for i in g) == list(range(6)), \
            "groups must partition legs 0..5"
        # Phase cycle: K x (shift, swing group_k, dwell). Fractions are
        # of the WHOLE period; each subcycle spans 1/K of it (for the
        # tripod K=2 this reduces exactly to the original math).
        sub = 1.0 / k
        shift_frac = _clip(float(shift_frac), 0.02, 0.8 * sub)
        swing_frac = _clip(float(swing_frac), 0.04,
                           sub - shift_frac - 0.005)
        dwell_frac = sub - shift_frac - swing_frac
        self._shifts = tuple(3 * j for j in range(k))
        self._swings = {3 * j + 1: self.groups[j] for j in range(k)}
        self._names = []
        for g in self.groups:
            legs = ",".join(str(i) for i in g)
            self._names += ["shift", f"swing ({legs})", "dwell"]
        self._names = tuple(self._names)
        self._durations = [
            f * self.period
            for f in (shift_frac, swing_frac, dwell_frac) * k]
        self.vx = vx
        self.vy = vy
        self.omega = omega
        self.leg_angles = [(i + 0.5) * math.pi / 3.0 for i in range(6)]
        self.sync_plant_stance()

        # Dead-reckoned body pose in the gait's world frame.
        self.px = 0.0
        self.py = 0.0
        self.pyaw = 0.0
        self._reset_anchors()

        self._last_t: float | None = None
        self._phase_idx = 0
        self._phase_time = 0.0
        self._nshift = 0        # shifts started; motion live from the 2nd
        self._start_phase()

    @classmethod
    def clamp_fit(cls, **kw) -> "NoSlipGait":
        """The 'cleanest under the servo clamp' preset (08-12 sweep)."""
        return cls(**{**cls.CLAMP_FIT_KW, **kw})

    @classmethod
    def ripple(cls, **kw) -> "NoSlipGait":
        """Classic ripple: opposite pairs swing, 4 feet always planted."""
        return cls(**{**cls.RIPPLE_KW, "groups": cls.GROUPS_RIPPLE, **kw})

    @classmethod
    def wave(cls, **kw) -> "NoSlipGait":
        """Classic wave: one leg at a time, 5 feet always planted."""
        return cls(**{**cls.WAVE_KW, "groups": cls.GROUPS_WAVE, **kw})

    def _reset_anchors(self) -> None:
        """Re-pin every foot at neutral under the current body pose."""
        # World XY anchor per foot (z is always the ground plane).
        self.anchors = [list(self._neutral_world(i)) for i in range(6)]
        self._swing: dict[int, tuple[float, float, float, float]] = {}
        self._last_q: list[tuple[float, float, float]] = [
            self._fallback for _ in range(6)]

    # ------------------------------------------------------------------
    # Stance geometry (same plant convention as TripodGait).
    def sync_plant_stance(self, hip_deg: float | None = None,
                          knee_deg: float | None = None) -> None:
        if hip_deg is None or knee_deg is None:
            h, k = _plant_hip_knee_deg()
            hip_deg = h if hip_deg is None else hip_deg
            knee_deg = k if knee_deg is None else knee_deg
        self.plant_hip_deg = float(hip_deg)
        self.plant_knee_deg = float(knee_deg)
        p = math.radians(self.plant_hip_deg)
        self.foot_neutral_x, self.foot_neutral_z = foot_rz_from_hip_knee(
            self.plant_hip_deg, self.plant_knee_deg)
        self._foot_radius = LEG_RADIAL + self.foot_neutral_x
        self._fallback = (0.0, p, math.radians(self.plant_knee_deg))
        # The world anchors are derived from this geometry: re-pin them
        # (a mid-walk call therefore snaps the footprint back to neutral;
        # set the stance BEFORE starting to walk).
        if hasattr(self, "anchors"):
            self._reset_anchors()

    def set_velocity(self, *, vx=None, vy=None, omega=None) -> None:
        """Takes effect at the next phase boundary (anchors never move)."""
        if vx is not None:
            self.vx = _clip(float(vx), -self.MAX_VX, self.MAX_VX)
        if vy is not None:
            self.vy = _clip(float(vy), -self.MAX_VY, self.MAX_VY)
        if omega is not None:
            self.omega = _clip(float(omega), -self.MAX_OMEGA, self.MAX_OMEGA)

    def set_alpha(self, alpha: float) -> None:
        """0 = step-then-shift .. 1 = continuous body motion.

        Takes effect at the next phase boundary (like set_velocity), so
        it never teleports the body or the pinned anchors mid-phase.
        """
        self.alpha = _clip(float(alpha), 0.0, 1.0)

    def set_lift_mm(self, lift_mm: float) -> None:
        """Swing-foot lift in mm (TripodGait call-surface parity)."""
        self.lift = _clip(float(lift_mm) * 0.001, 0.005, 0.05)

    def reset_phase(self, *, phase: float = 0.0, t: float = 0.0) -> None:
        """Restart the cycle and re-pin every foot at neutral under the
        current body pose (TripodGait call-surface parity; the cycle
        always restarts at the first shift, ``phase`` is ignored)."""
        del phase
        self._last_t = t if t else None
        self._phase_idx = 0
        self._phase_time = 0.0
        self._nshift = 0
        self._reset_anchors()
        self._start_phase()

    def stop(self) -> None:
        self.set_velocity(vx=0.0, vy=0.0, omega=0.0)

    def commanded_pose(self) -> tuple[float, float, float]:
        """Dead-reckoned (x, y, yaw) of the body in the gait world frame."""
        return self.px, self.py, self.pyaw

    def phase_name(self) -> str:
        return self._names[self._phase_idx]

    # ------------------------------------------------------------------
    # Internals.
    def _neutral_world(self, i: int) -> tuple[float, float]:
        """Neutral (yaw=0, plant hip/knee) foot under the CURRENT pose."""
        a = self.leg_angles[i]
        ca, sa = math.cos(a), math.sin(a)
        # Yaw origin + the offset arm (foot_neutral_x, HIP_Y) at yaw=0.
        nx = LEG_RADIAL * ca + self.foot_neutral_x * ca - HIP_Y * sa
        ny = LEG_RADIAL * sa + self.foot_neutral_x * sa + HIP_Y * ca
        c, s = math.cos(self.pyaw), math.sin(self.pyaw)
        return (self.px + c * nx - s * ny, self.py + s * nx + c * ny)

    def _cycle_twist(self) -> tuple[float, float, float]:
        """Clamped body-frame twist for one full cycle."""
        vx = _clip(self.vx, -self.MAX_VX, self.MAX_VX)
        vy = _clip(self.vy, -self.MAX_VY, self.MAX_VY)
        om = _clip(self.omega, -self.MAX_OMEGA, self.MAX_OMEGA)
        dx, dy = vx * self.period, vy * self.period
        stride = math.hypot(dx, dy)
        if stride > self.STRIDE_MAX:
            k = self.STRIDE_MAX / stride
            dx *= k
            dy *= k
        dyaw = _clip(om * self.period, -self.YAW_STEP_MAX, self.YAW_STEP_MAX)
        return dx, dy, dyaw

    def _advance_pose(self, dx: float, dy: float, dyaw: float) -> None:
        """Apply a body-frame displacement with midpoint yaw rotation."""
        c = math.cos(self.pyaw + 0.5 * dyaw)
        s = math.sin(self.pyaw + 0.5 * dyaw)
        self.px += c * dx - s * dy
        self.py += s * dx + c * dy
        self.pyaw += dyaw

    def _swing_target_advance(self) -> float:
        """Fraction of the cycle twist from swing start to the MIDDLE of
        the swing feet's next planted interval (the anchor symmetry
        point). Walks the coming five planted phases so startup phases
        (before the second shift, which command zero body motion) are
        skipped. At steady state this is 0.5 at alpha=0 and
        0.5 + alpha*swing_frac/2 in general: the alpha drift keeps the
        body moving during the swing itself, pushing the touchdown pose
        forward before the planted interval even begins.
        """
        a = self.alpha
        n = len(self._names)
        nshift = self._nshift
        live = nshift >= 2
        swing = (a * self._durations[self._phase_idx] / self.period
                 if live else 0.0)
        pulse = (1.0 - a) / len(self._shifts)   # ease share per shift
        planted = 0.0
        for k in range(1, n):          # the n-1 phases the feet stay down
            j = (self._phase_idx + k) % n
            if j in self._shifts:
                nshift += 1
                live = nshift >= 2
            if not live:
                continue
            planted += a * self._durations[j] / self.period
            if j in self._shifts:
                planted += pulse
        return swing + 0.5 * planted

    def _start_phase(self) -> None:
        idx = self._phase_idx
        if idx in self._shifts:
            self._nshift += 1
        # No body motion before the second shift of a run (feet start
        # pinned at neutral), so neither tripod exceeds ~half a stride
        # of excursion at startup.
        live = self._nshift >= 2
        dx, dy, dyaw = self._cycle_twist()
        a = self.alpha
        # alpha's share of the twist drifts at a constant rate through
        # EVERY phase; captured here so set_velocity / set_alpha take
        # effect at phase boundaries.
        self._lin_rate = ((a * dx / self.period, a * dy / self.period,
                           a * dyaw / self.period)
                          if live else (0.0, 0.0, 0.0))
        if idx in self._shifts:                # shift: capture ease pulse
            s = (1.0 - a) / len(self._shifts) if live else 0.0
            self._shift_twist = (s * dx, s * dy, s * dyaw)
        elif idx in self._swings:              # swing: plan the step
            tripod = self._swings[idx]
            f = self._swing_target_advance()
            saved = (self.px, self.py, self.pyaw)
            self._advance_pose(f * dx, f * dy, f * dyaw)
            self._swing = {}
            for i in tripod:
                tx, ty = self._neutral_world(i)
                ax, ay = self.anchors[i]
                self._swing[i] = (ax, ay, tx, ty)
            self.px, self.py, self.pyaw = saved

    def _end_phase(self) -> None:
        if self._phase_idx in self._swings:
            for i, (_ax, _ay, tx, ty) in self._swing.items():
                self.anchors[i] = [tx, ty]
            self._swing = {}

    def _integrate(self, dt_s: float) -> None:
        """Integrate the machine forward, crossing phase boundaries."""
        remaining = dt_s
        while remaining > 1e-12:
            dur = self._durations[self._phase_idx]
            left = dur - self._phase_time
            step = min(remaining, left)
            if self._phase_idx in self._shifts and dur > 0.0:
                p0 = self._phase_time / dur
                p1 = (self._phase_time + step) / dur
                ds = _ease(p1) - _ease(p0)
                hx, hy, hyaw = self._shift_twist
                self._advance_pose(hx * ds, hy * ds, hyaw * ds)
            lx, ly, lyaw = self._lin_rate
            if lx or ly or lyaw:               # alpha drift (all phases)
                self._advance_pose(lx * step, ly * step, lyaw * step)
            self._phase_time += step
            remaining -= step
            if dur - self._phase_time <= 1e-12:
                self._end_phase()
                self._phase_idx = (self._phase_idx + 1) % len(self._names)
                self._phase_time = 0.0
                self._start_phase()

    def _foot_body(self, i: int) -> tuple[float, float, float]:
        """Commanded foot position in the body frame (m)."""
        if i in self._swing:
            dur = self._durations[self._phase_idx]
            p = self._phase_time / dur if dur > 0 else 1.0
            s = _minjerk(p)
            ax, ay, tx, ty = self._swing[i]
            wx = ax + (tx - ax) * s
            wy = ay + (ty - ay) * s
            dz = self.lift * math.sin(math.pi * s)
        else:
            wx, wy = self.anchors[i]
            dz = 0.0
        c, s_ = math.cos(self.pyaw), math.sin(self.pyaw)
        rx, ry = wx - self.px, wy - self.py
        bx = c * rx + s_ * ry
        by = -s_ * rx + c * ry
        return bx, by, self.foot_neutral_z + dz

    def _leg_joints(self, i: int, fx: float, fy: float, fz: float
                    ) -> tuple[float, float, float]:
        a = self.leg_angles[i]
        ox = LEG_RADIAL * math.cos(a)
        oy = LEG_RADIAL * math.sin(a)
        rx, ry = fx - ox, fy - oy
        ca, sa = math.cos(a), math.sin(a)
        x_yaw = ca * rx + sa * ry
        y_yaw = -sa * rx + ca * ry
        # Offset-arm yaw: the leg plane rides at lateral HIP_Y, so
        # foot = R(yaw) @ (r, HIP_Y); solve r and yaw exactly.
        d2 = x_yaw * x_yaw + y_yaw * y_yaw
        if d2 <= HIP_Y * HIP_Y + 1e-9:
            return self._last_q[i]
        r = math.sqrt(d2 - HIP_Y * HIP_Y)
        yaw = math.atan2(y_yaw, x_yaw) - math.atan2(HIP_Y, r)
        ik = _leg_ik((r, 0.0, fz))
        if ik is None:
            return self._last_q[i]
        hip, knee = ik
        q = (_clip(yaw, -YAW_LIM, YAW_LIM),
             _clip(hip, HIP_LIM[0], HIP_LIM[1]),
             _clip(knee, KNEE_LIM[0], KNEE_LIM[1]))
        self._last_q[i] = q
        return q

    # ------------------------------------------------------------------
    def desired_deg(self, t: float) -> list[float]:
        """18 joint angles (deg) for time ``t`` (s); same order as qpos."""
        if self._last_t is None:
            self._last_t = t
        dt = max(0.0, t - self._last_t)
        self._last_t = t
        self._integrate(dt)
        out: list[float] = []
        for i in range(6):
            fx, fy, fz = self._foot_body(i)
            yaw, hip, knee = self._leg_joints(i, fx, fy, fz)
            out.extend([math.degrees(yaw), math.degrees(hip),
                        math.degrees(knee)])
        return out
