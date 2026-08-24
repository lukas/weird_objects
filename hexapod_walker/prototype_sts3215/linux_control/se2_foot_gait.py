"""SE(2) foot-space gait: per-leg neutrals + workspaces, Bézier swings.

``SE2FootGait`` is a rules-based walking controller commanded by a planar
body twist (vx, vy, wz).  It shares ``NoSlipGait``'s core guarantee —
planted feet are pinned to fixed anchors in a dead-reckoned world frame,
so their commanded ground-relative velocity is exactly zero — but is a
different machine:

  neutrals     six LEG-SPECIFIC neutral foot positions in the body frame
               (plant-pose default + optional per-leg radial/tangential
               offsets), each with a numerically computed SAFE WORKSPACE
               disc: the largest circle around the neutral, at stance
               height, whose every point passes strict fixed-branch IK
               inside the joint limits (times a safety factor).
  stance       inverse body motion: the body pose integrates the (exact
               SE(2) exponential of the) commanded twist and each planted
               foot tracks  g(t)^-1 · anchor  — never a body-frame drag.
  swing        a single degree-5 3-D Bézier per step: double control
               points at both ends give ZERO world-frame velocity at
               liftoff and touchdown (no scuff even while the body
               moves); the z control points give a lift-high arc with
               apex exactly ``lift`` at mid-swing.  (NoSlipGait uses
               min-jerk XY + half-sine z, which lands with nonzero
               vertical rate; the Bézier does not.)
  scheduler    phase-offset windows, continuous body motion throughout
               (no shift/dwell machine):
                 TETRAPOD  ((0,3), (1,4), (2,5))  2 swing / 4 down
                 WAVE      ((0,),(3,),(1,),(4,),(2,),(5,))  1 / 5 down
               Each group's swing window is centred in its 1/K slice of
               the cycle, so there is a brief all-feet-down dwell
               between consecutive swings.
  support      per-tick support-polygon check: signed distance from the
               CoM ground projection (body origin + ``com_offset``) to
               the convex hull of the planted feet.  ``min_support_margin``
               accumulates the worst value since the last reset.
  scaling      commands are scaled DOWN (uniformly, preserving path
               curvature) to the largest factor k <= 1 whose steady-state
               stance trajectories stay inside every leg's safe workspace
               and pass strict IK — including lift clearance at the
               stride extremes.  ``last_command_scale`` reports k.
  IK           fixed positive-knee branch only (tripod_gait._leg_ik) with
               the exact HIP_Y offset-arm yaw solve (noslip_gait); the
               branch never flips mid-gait.

Leg numbering: 0 front-left, 1 mid-left, 2 rear-left, 3 rear-right,
4 mid-right, 5 front-right.  Angles: joint = leg*3 + (yaw, hip, knee),
degrees out of ``desired_deg``; positive hip/knee move the foot down.

Call surface matches TripodGait/NoSlipGait (``desired_deg(t)``,
``set_velocity(*, vx=, vy=, omega=)``, ``sync_plant_stance``,
``set_lift_mm``, ``reset_phase``, ``stop``); registered in
``drive_controller`` as GAIT 4 (tetrapod) and GAIT 5 (wave).
Stdlib-only, vendorable to the Uno Q.  The caller stops ticking
``desired_deg`` when idle (as drive_controller does in "stand" mode) —
with a zero command the scheduler still lifts feet in place, like the
other gaits.

Grid evaluation over all planar commands (kinematic, no sim):

    uv run python linux_control/se2_foot_gait.py                # default 5^3 grid
    uv run python linux_control/se2_foot_gait.py --gait wave --n 3
    uv run python linux_control/se2_foot_gait.py --cmd 0.01,0,0.05 --out out.json

Sim replay (full servo/safety stack) reuses the verify_noslip harness:
build the gait and pass it to ``rl_move.sim.verify_noslip.rollout``.
"""
from __future__ import annotations

import math

from noslip_gait import HIP_LIM, HIP_Y, KNEE_LIM, YAW_LIM
from tripod_gait import (COXA, FEMUR, TIBIA, LEG_RADIAL, _clip, _leg_ik,
                         _plant_hip_knee_deg, foot_rz_from_hip_knee)


# ---------------------------------------------------------------------------
# SE(2) helpers.
def se2_exp(vx: float, vy: float, wz: float, t: float
            ) -> tuple[float, float, float]:
    """Displacement (dx, dy, dyaw) in the START frame after riding the
    constant body twist (vx, vy, wz) for ``t`` seconds (exact)."""
    th = wz * t
    if abs(wz) < 1e-9:
        return vx * t, vy * t, th
    s, c = math.sin(th), math.cos(th)
    ax, ay = vx / wz, vy / wz
    return ax * s - ay * (1.0 - c), ax * (1.0 - c) + ay * s, th


def se2_log(dx: float, dy: float, dth: float) -> tuple[float, float, float]:
    """Inverse of ``se2_exp`` with t=1: the (vx*t, vy*t, wz*t) twist whose
    exponential is the given start-frame displacement."""
    if abs(dth) < 1e-9:
        return dx, dy, dth
    s, c = math.sin(dth), math.cos(dth)
    k = dth / (2.0 * (1.0 - c))
    return (k * (s * dx + (1.0 - c) * dy),
            k * (-(1.0 - c) * dx + s * dy), dth)


# ---------------------------------------------------------------------------
# Bézier swing.
def bezier(ctrl: list[float], tau: float) -> float:
    """De Casteljau evaluation of a 1-D Bézier at tau in [0, 1]."""
    tau = _clip(tau, 0.0, 1.0)
    pts = list(ctrl)
    for r in range(1, len(pts)):
        for j in range(len(pts) - r):
            pts[j] = pts[j] * (1.0 - tau) + pts[j + 1] * tau
    return pts[0]


# z control points [0, 0, c, c, 0, 0]: apex = 0.625*c at tau=0.5, so
# c = 1.6*lift puts the apex exactly at ``lift``; doubled end points give
# zero vertical velocity at liftoff AND touchdown.
_Z_APEX_GAIN = 1.6


def swing_point(ax: float, ay: float, tx: float, ty: float,
                lift: float, tau: float) -> tuple[float, float, float]:
    """World (x, y, dz) on the degree-5 Bézier step from (ax, ay) to
    (tx, ty) with apex height ``lift``.  Double control points at both
    ends -> zero world velocity at tau = 0 and tau = 1."""
    fx = [ax, ax, ax + (tx - ax) / 3.0, ax + 2.0 * (tx - ax) / 3.0, tx, tx]
    fy = [ay, ay, ay + (ty - ay) / 3.0, ay + 2.0 * (ty - ay) / 3.0, ty, ty]
    c = _Z_APEX_GAIN * lift
    fz = [0.0, 0.0, c, c, 0.0, 0.0]
    return bezier(fx, tau), bezier(fy, tau), bezier(fz, tau)


# ---------------------------------------------------------------------------
# Support polygon (stdlib port of the sim's convex-hull signed margin).
def convex_hull(points: list[tuple[float, float]]
                ) -> list[tuple[float, float]]:
    """Monotone-chain convex hull, counter-clockwise, no duplicates."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return ((a[0] - o[0]) * (b[1] - o[1])
                - (a[1] - o[1]) * (b[0] - o[0]))

    lower: list[tuple[float, float]] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper: list[tuple[float, float]] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def support_margin(stance_xy: list[tuple[float, float]],
                   com_xy: tuple[float, float] = (0.0, 0.0)) -> float:
    """Signed distance (m) from the CoM ground projection to the support
    polygon boundary; positive inside, -inf with < 3 stance feet."""
    hull = convex_hull(list(stance_xy))
    if len(hull) < 3:
        return float("-inf")
    cx, cy = com_xy
    margin = float("inf")
    n = len(hull)
    for j in range(n):
        ax, ay = hull[j]
        bx, by = hull[(j + 1) % n]
        ex, ey = bx - ax, by - ay
        length = math.hypot(ex, ey)
        if length < 1e-12:
            continue
        margin = min(margin, (ex * (cy - ay) - ey * (cx - ax)) / length)
    return margin


def _smoothstep(p: float) -> float:
    p = _clip(p, 0.0, 1.0)
    return p * p * (3.0 - 2.0 * p)


class SE2FootGait:
    """Planar-twist hexapod gait on per-leg foot workspaces.

    Timing: the cycle is split into K = len(groups) equal slices; group k
    swings in a window of width ``swing_frac`` (fraction of the WHOLE
    period) centred in slice k.  Every leg is planted for
    duty = 1 - swing_frac of the cycle.  Body motion is continuous
    (constant twist, exact SE(2) integration) with a smoothstep ramp over
    the first cycle so startup leg excursions stay within the same
    half-stride budget as steady state.

    Touchdown targets are frozen at liftoff: the target is the leg's
    neutral position under the body pose predicted (current filtered
    twist) at the MIDDLE of the foot's next planted interval, so stance
    excursions stay symmetric about the neutral.  Command changes are
    low-passed (``cmd_tau``) and never move a pinned anchor.
    """

    MAX_VX = 0.040       # m/s   (hard caps; workspace scaling refines)
    MAX_VY = 0.035
    MAX_OMEGA = 0.30     # rad/s
    RAMP_CYCLES = 1.5    # startup speed ramp; keeps the pre-first-swing
    #                      drift of the last-scheduled legs well inside
    #                      the half-stride budget the scaling assumes.

    GROUPS_TETRAPOD = ((0, 3), (1, 4), (2, 5))
    GROUPS_WAVE = ((0,), (3,), (1,), (4,), (2,), (5,))

    # Timing follows the clamp-fit philosophy (noslip RIPPLE_KW/WAVE_KW):
    # swing durations ~2.4 s / ~3.1 s fit the ~31 deg/s servo cruise.
    TETRAPOD_KW = dict(period=8.0, swing_frac=0.30, lift=0.020)
    WAVE_KW = dict(period=20.0, swing_frac=0.155, lift=0.018)

    def __init__(
        self,
        *,
        gait: str = "tetrapod",
        groups: tuple | None = None,
        period: float | None = None,
        swing_frac: float | None = None,
        lift: float | None = None,
        vx: float = 0.0,
        vy: float = 0.0,
        omega: float = 0.0,
        neutral_offsets: list | None = None,
        workspace_margin: float = 0.9,
        cmd_tau: float = 0.4,
        com_offset: tuple[float, float] = (0.0, 0.0),
    ):
        presets = {"tetrapod": (self.GROUPS_TETRAPOD, self.TETRAPOD_KW),
                   "wave": (self.GROUPS_WAVE, self.WAVE_KW)}
        if gait not in presets:
            raise ValueError(f"gait must be one of {sorted(presets)}")
        self.gait_name = gait
        base_groups, kw = presets[gait]
        self.groups = tuple(tuple(int(i) for i in g)
                            for g in (groups or base_groups))
        assert sorted(i for g in self.groups for i in g) == list(range(6)), \
            "groups must partition legs 0..5"
        self.period = max(float(period if period is not None
                                else kw["period"]), 0.4)
        k = len(self.groups)
        sub = 1.0 / k
        sf = float(swing_frac if swing_frac is not None else kw["swing_frac"])
        self.swing_frac = _clip(sf, 0.02, 0.9 * sub)
        self.duty = 1.0 - self.swing_frac
        # Swing window start per leg, centred in the group's cycle slice.
        self._win: dict[int, float] = {}
        for gi, g in enumerate(self.groups):
            start = gi * sub + 0.5 * (sub - self.swing_frac)
            for i in g:
                self._win[i] = start
        self._events = sorted({o for o in self._win.values()}
                              | {o + self.swing_frac
                                 for o in self._win.values()})

        self.lift = _clip(float(lift if lift is not None
                                else kw["lift"]), 0.005, 0.05)
        self.leg_angles = [(i + 0.5) * math.pi / 3.0 for i in range(6)]
        self.neutral_offsets = [
            (float(o[0]), float(o[1]))
            for o in (neutral_offsets or [(0.0, 0.0)] * 6)]
        assert len(self.neutral_offsets) == 6
        self.workspace_margin = _clip(float(workspace_margin), 0.1, 1.0)
        self.cmd_tau = max(float(cmd_tau), 1e-3)
        self.com_offset = (float(com_offset[0]), float(com_offset[1]))

        self.ik_failures = 0
        self.limit_clips = 0
        self.min_support_margin = float("inf")
        self.last_support_margin: float | None = None

        self.sync_plant_stance()

        # Dead-reckoned body pose in the gait's world frame.
        self.px = 0.0
        self.py = 0.0
        self.pyaw = 0.0
        self._reset_anchors()

        self._u = 0.0                    # cycle phase in [0, 1)
        self._elapsed = 0.0
        self._last_t: float | None = None
        self._filt = [0.0, 0.0, 0.0]     # filtered scaled twist
        self._req: tuple | None = None   # last requested (memo)
        self._target = (0.0, 0.0, 0.0)   # scaled target twist
        self.last_command_scale = 1.0
        self.set_velocity(vx=vx, vy=vy, omega=omega)

    @classmethod
    def tetrapod(cls, **kw) -> "SE2FootGait":
        return cls(gait="tetrapod", **kw)

    @classmethod
    def wave(cls, **kw) -> "SE2FootGait":
        return cls(gait="wave", **kw)

    # ------------------------------------------------------------------
    # Stance geometry: leg-specific neutrals + safe workspace discs.
    def sync_plant_stance(self, hip_deg: float | None = None,
                          knee_deg: float | None = None) -> None:
        """Set foot height / neutrals from the stand-plant hip & knee.

        Rebuilds the per-leg neutral positions and safe-workspace radii
        and re-pins the anchors — set the stance BEFORE walking, a
        mid-walk call snaps the footprint back to neutral."""
        if hip_deg is None or knee_deg is None:
            h, kn = _plant_hip_knee_deg()
            hip_deg = h if hip_deg is None else hip_deg
            knee_deg = kn if knee_deg is None else knee_deg
        self.plant_hip_deg = float(hip_deg)
        self.plant_knee_deg = float(knee_deg)
        p = math.radians(self.plant_hip_deg)
        self.foot_neutral_x, self.foot_neutral_z = foot_rz_from_hip_knee(
            self.plant_hip_deg, self.plant_knee_deg)
        self._fallback = (0.0, p, math.radians(self.plant_knee_deg))
        self._rebuild_geometry()
        if hasattr(self, "anchors"):
            self._reset_anchors()

    def set_neutral_offsets(self, offsets: list) -> None:
        """Per-leg (radial, tangential) neutral offsets in metres, in the
        leg's azimuth frame.  Rebuilds workspaces and re-pins anchors."""
        assert len(offsets) == 6
        self.neutral_offsets = [(float(o[0]), float(o[1])) for o in offsets]
        self._rebuild_geometry()
        if hasattr(self, "anchors"):
            self._reset_anchors()

    def _rebuild_geometry(self) -> None:
        self.neutral_body: list[tuple[float, float]] = []
        for i in range(6):
            a = self.leg_angles[i]
            ca, sa = math.cos(a), math.sin(a)
            dr, dt = self.neutral_offsets[i]
            r = LEG_RADIAL + self.foot_neutral_x + dr
            t = HIP_Y + dt
            self.neutral_body.append((r * ca - t * sa, r * sa + t * ca))
        self.workspace_r = [self._workspace_radius(i) for i in range(6)]

    def _workspace_radius(self, i: int) -> float:
        """Largest disc around leg i's neutral (at stance height) whose
        boundary passes strict fixed-branch IK, times the safety factor."""
        nx, ny = self.neutral_body[i]
        z = self.foot_neutral_z

        def ok(r: float) -> bool:
            for m in range(12):
                th = m * math.pi / 6.0
                q = self.leg_ik_body(i, nx + r * math.cos(th),
                                     ny + r * math.sin(th), z, strict=True)
                if q is None:
                    return False
            return True

        if not ok(0.0):
            return 0.0        # degenerate plant: neutral itself infeasible
        lo, hi = 0.0, 0.15    # 150 mm safely exceeds any reach
        for _ in range(22):
            mid = 0.5 * (lo + hi)
            if ok(mid):
                lo = mid
            else:
                hi = mid
        return lo * self.workspace_margin

    # ------------------------------------------------------------------
    # Fixed-branch IK / FK (exact HIP_Y offset arm, positive-knee branch).
    def leg_ik_body(self, i: int, fx: float, fy: float, fz: float,
                    *, strict: bool = False
                    ) -> tuple[float, float, float] | None:
        """(yaw, hip, knee) rad for a body-frame foot target, or None.

        Always the positive-knee branch (never flips).  ``strict`` also
        requires the solution inside the joint limits."""
        a = self.leg_angles[i]
        ca, sa = math.cos(a), math.sin(a)
        rx = fx - LEG_RADIAL * ca
        ry = fy - LEG_RADIAL * sa
        x_yaw = ca * rx + sa * ry
        y_yaw = -sa * rx + ca * ry
        d2 = x_yaw * x_yaw + y_yaw * y_yaw
        if d2 <= HIP_Y * HIP_Y + 1e-9:
            return None
        r = math.sqrt(d2 - HIP_Y * HIP_Y)
        yaw = math.atan2(y_yaw, x_yaw) - math.atan2(HIP_Y, r)
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        ik = _leg_ik((r, 0.0, fz))
        if ik is None:
            return None
        hip, knee = ik
        if strict and not (-YAW_LIM <= yaw <= YAW_LIM
                           and HIP_LIM[0] <= hip <= HIP_LIM[1]
                           and KNEE_LIM[0] <= knee <= KNEE_LIM[1]):
            return None
        return yaw, hip, knee

    def fk_foot_body(self, i: int, yaw: float, hip: float, knee: float
                     ) -> tuple[float, float, float]:
        """Body-frame foot position for one leg (inverse of leg_ik_body)."""
        r = COXA + FEMUR * math.cos(hip) + TIBIA * math.cos(knee)
        z = -FEMUR * math.sin(hip) - TIBIA * math.sin(knee)
        cy, sy = math.cos(yaw), math.sin(yaw)
        x_yaw = r * cy - HIP_Y * sy
        y_yaw = r * sy + HIP_Y * cy
        a = self.leg_angles[i]
        ca, sa = math.cos(a), math.sin(a)
        return (LEG_RADIAL * ca + ca * x_yaw - sa * y_yaw,
                LEG_RADIAL * sa + sa * x_yaw + ca * y_yaw, z)

    def foot_feasible(self, i: int, fx: float, fy: float, fz: float) -> bool:
        """Strict IK feasibility AND inside leg i's safe workspace disc."""
        nx, ny = self.neutral_body[i]
        if math.hypot(fx - nx, fy - ny) > self.workspace_r[i]:
            return False
        return self.leg_ik_body(i, fx, fy, fz, strict=True) is not None

    # ------------------------------------------------------------------
    # Command interface.
    def set_velocity(self, *, vx=None, vy=None, omega=None) -> None:
        """Set the planar twist; scaled for workspace feasibility.

        The feasibility search is a few ms of IK sampling, so repeated
        calls with an unchanged command are memoized."""
        cur = self._req or (0.0, 0.0, 0.0)
        vx = cur[0] if vx is None else _clip(float(vx),
                                             -self.MAX_VX, self.MAX_VX)
        vy = cur[1] if vy is None else _clip(float(vy),
                                             -self.MAX_VY, self.MAX_VY)
        om = cur[2] if omega is None else _clip(float(omega),
                                                -self.MAX_OMEGA,
                                                self.MAX_OMEGA)
        req = (vx, vy, om)
        if req == self._req:
            return
        self._req = req
        k, tw = self._feasible_twist(*req)
        self.last_command_scale = k
        self._target = tw

    def set_lift_mm(self, lift_mm: float) -> None:
        self.lift = _clip(float(lift_mm) * 0.001, 0.005, 0.05)

    def stop(self) -> None:
        self.set_velocity(vx=0.0, vy=0.0, omega=0.0)

    def reset_phase(self, *, phase: float = 0.0, t: float = 0.0) -> None:
        """Restart the cycle and re-pin every foot at neutral under the
        current body pose (call-surface parity; ``phase`` is ignored —
        the cycle always restarts at the head of the schedule)."""
        del phase
        self._last_t = t if t else None
        self._u = 0.0
        self._elapsed = 0.0
        self._filt = [0.0, 0.0, 0.0]
        self.ik_failures = 0
        self.limit_clips = 0
        self.min_support_margin = float("inf")
        self.last_support_margin = None
        self._reset_anchors()

    def commanded_pose(self) -> tuple[float, float, float]:
        """Dead-reckoned (x, y, yaw) of the body in the gait world frame."""
        return self.px, self.py, self.pyaw

    def phase_name(self) -> str:
        if not self._swing:
            return "stance"
        legs = ",".join(str(i) for i in sorted(self._swing))
        return f"swing ({legs})"

    def stance_legs(self) -> tuple[int, ...]:
        return tuple(i for i in range(6) if i not in self._swing)

    def support_margin_now(self) -> float | None:
        return self.last_support_margin

    # ------------------------------------------------------------------
    # Command scaling for workspace feasibility.
    def _feasible_twist(self, vx: float, vy: float, wz: float
                        ) -> tuple[float, tuple[float, float, float]]:
        """Largest k <= 1 with a feasible steady-state cycle for k*twist."""
        if max(abs(vx), abs(vy), abs(wz)) < 1e-9:
            return 1.0, (0.0, 0.0, 0.0)
        if self._steady_ok(vx, vy, wz):
            return 1.0, (vx, vy, wz)
        lo, hi = 0.0, 1.0
        for _ in range(16):
            mid = 0.5 * (lo + hi)
            if self._steady_ok(vx * mid, vy * mid, wz * mid):
                lo = mid
            else:
                hi = mid
        return lo, (vx * lo, vy * lo, wz * lo)

    def _steady_ok(self, vx: float, vy: float, wz: float) -> bool:
        """Steady-state cycle feasibility for one candidate twist.

        Reference frame: body pose at the middle of a planted interval.
        The anchor sits at the leg's neutral there, and the stance foot
        in the body frame is g(tau)^-1 . neutral for tau in
        [-T_st/2, +T_st/2].  Every stance sample must stay inside the
        safe workspace disc and pass strict IK.

        The SWING is then simulated exactly as the runtime executes it:
        the world-frame Bézier runs from this anchor to the neutral under
        the pose one full cycle ahead, while the body keeps riding the
        twist — early in the swing the Bézier progress is slow (doubled
        control points), so the body-frame excursion exceeds the stance
        extremes; every sample must pass strict IK at its lifted height.
        """
        t_st = self.duty * self.period
        half = 0.5 * t_st
        t_sw = self.swing_frac * self.period
        z = self.foot_neutral_z
        n_st, n_sw = 13, 9
        for i in range(6):
            nx, ny = self.neutral_body[i]
            rad = self.workspace_r[i]
            for j in range(n_st):
                tau = -half + t_st * j / (n_st - 1)
                dx, dy, dth = se2_exp(vx, vy, wz, tau)
                c, s = math.cos(dth), math.sin(dth)
                rx, ry = nx - dx, ny - dy
                px = c * rx + s * ry
                py = -s * rx + c * ry
                if math.hypot(px - nx, py - ny) > rad:
                    return False
                if self.leg_ik_body(i, px, py, z, strict=True) is None:
                    return False
            # Next anchor: neutral under the pose one cycle ahead
            # (consecutive mid-stances are exactly one period apart).
            dx, dy, dth = se2_exp(vx, vy, wz, self.period)
            c, s = math.cos(dth), math.sin(dth)
            tx = dx + c * nx - s * ny
            ty = dy + s * nx + c * ny
            for j in range(n_sw):
                sfr = j / (n_sw - 1)
                wx, wy, dz = swing_point(nx, ny, tx, ty, self.lift, sfr)
                tau = half + sfr * t_sw
                bx, by, bth = se2_exp(vx, vy, wz, tau)
                cb, sb = math.cos(bth), math.sin(bth)
                rx, ry = wx - bx, wy - by
                px = cb * rx + sb * ry
                py = -sb * rx + cb * ry
                if self.leg_ik_body(i, px, py, z + dz, strict=True) is None:
                    return False
        return True

    # ------------------------------------------------------------------
    # Phase machine.
    def _reset_anchors(self) -> None:
        self.anchors = [list(self._neutral_world(i, self.px, self.py,
                                                 self.pyaw))
                        for i in range(6)]
        self._swing: dict[int, tuple[float, float, float, float]] = {}
        self._last_q: list[tuple[float, float, float]] = [
            self._fallback for _ in range(6)]

    def _neutral_world(self, i: int, px: float, py: float, pyaw: float
                       ) -> tuple[float, float]:
        nx, ny = self.neutral_body[i]
        c, s = math.cos(pyaw), math.sin(pyaw)
        return px + c * nx - s * ny, py + s * nx + c * ny

    def _advance_pose(self, dt_s: float) -> None:
        dx, dy, dth = se2_exp(self._filt[0], self._filt[1],
                              self._filt[2], dt_s)
        c, s = math.cos(self.pyaw), math.sin(self.pyaw)
        self.px += c * dx - s * dy
        self.py += s * dx + c * dy
        self.pyaw += dth

    def _predict_pose(self, horizon: float) -> tuple[float, float, float]:
        """Body pose ``horizon`` seconds ahead, integrating the KNOWN
        deterministic future: startup ramp, command filter and target
        twist.  Assuming the current twist constant instead makes feet
        planted mid-ramp overshoot their stance budget by the speed
        gained during their (long) stance — measured +40% excursions."""
        n = max(8, int(horizon / 0.1))
        step = horizon / n
        vx, vy, wz = self._filt
        px, py, pyaw = self.px, self.py, self.pyaw
        elapsed = self._elapsed
        a = 1.0 - math.exp(-step / self.cmd_tau)
        for _ in range(n):
            elapsed += step
            ramp = _smoothstep(elapsed / (self.RAMP_CYCLES * self.period))
            vx += a * (self._target[0] * ramp - vx)
            vy += a * (self._target[1] * ramp - vy)
            wz += a * (self._target[2] * ramp - wz)
            dx, dy, dth = se2_exp(vx, vy, wz, step)
            c, s = math.cos(pyaw), math.sin(pyaw)
            px += c * dx - s * dy
            py += s * dx + c * dy
            pyaw += dth
        return px, py, pyaw

    def _plan_swing(self, i: int) -> None:
        """Freeze the step: liftoff anchor -> neutral under the body pose
        predicted at the middle of the foot's next planted interval."""
        ax, ay = self.anchors[i]
        horizon = (self.swing_frac + 0.5 * self.duty) * self.period
        ppx, ppy, ppyaw = self._predict_pose(horizon)
        tx, ty = self._neutral_world(i, ppx, ppy, ppyaw)
        self._swing[i] = (ax, ay, tx, ty)

    def _fire_events(self, u: float) -> None:
        for i, start in self._win.items():
            if abs(start - u) < 1e-9:
                self._plan_swing(i)
            elif abs(start + self.swing_frac - u) < 1e-9 and i in self._swing:
                _ax, _ay, tx, ty = self._swing.pop(i)
                self.anchors[i] = [tx, ty]

    def _advance(self, dt_s: float) -> None:
        remaining = dt_s
        guard = 0
        while remaining > 1e-12 and guard < 10000:
            guard += 1
            best_du, best_u = None, None
            for e in self._events:
                du = (e - self._u) % 1.0
                if du < 1e-9:
                    du = 1.0        # just fired: next occurrence next cycle
                if best_du is None or du < best_du:
                    best_du, best_u = du, e
            t_event = best_du * self.period
            if t_event <= remaining + 1e-12:
                self._advance_pose(t_event)
                self._u = best_u
                self._fire_events(best_u)
                remaining -= t_event
            else:
                self._advance_pose(remaining)
                self._u = (self._u + remaining / self.period) % 1.0
                remaining = 0.0

    # ------------------------------------------------------------------
    # Per-tick output.
    def _foot_body(self, i: int) -> tuple[float, float, float]:
        """Commanded foot position in the body frame (m)."""
        if i in self._swing:
            tau = ((self._u - self._win[i]) % 1.0) / self.swing_frac
            ax, ay, tx, ty = self._swing[i]
            wx, wy, dz = swing_point(ax, ay, tx, ty, self.lift, tau)
        else:
            wx, wy = self.anchors[i]
            dz = 0.0
        c, s = math.cos(self.pyaw), math.sin(self.pyaw)
        rx, ry = wx - self.px, wy - self.py
        return (c * rx + s * ry, -s * rx + c * ry,
                self.foot_neutral_z + dz)

    def _leg_joints(self, i: int, fx: float, fy: float, fz: float
                    ) -> tuple[float, float, float]:
        q = self.leg_ik_body(i, fx, fy, fz, strict=False)
        if q is None:
            self.ik_failures += 1
            return self._last_q[i]
        yaw, hip, knee = q
        cy = _clip(yaw, -YAW_LIM, YAW_LIM)
        ch = _clip(hip, HIP_LIM[0], HIP_LIM[1])
        ck = _clip(knee, KNEE_LIM[0], KNEE_LIM[1])
        if (abs(cy - yaw) > 1e-6 or abs(ch - hip) > 1e-6
                or abs(ck - knee) > 1e-6):
            self.limit_clips += 1
        self._last_q[i] = (cy, ch, ck)
        return self._last_q[i]

    def desired_deg(self, t: float) -> list[float]:
        """18 joint angles (deg) for time ``t`` (s); same order as qpos."""
        if self._last_t is None:
            self._last_t = t
        dt = max(0.0, t - self._last_t)
        self._last_t = t
        self._elapsed += dt
        # Startup speed ramp: bounds the drift of legs still waiting for
        # their first swing (and hence first touchdown lead) to well
        # inside the half-stride budget the workspace scaling assumes.
        ramp = _smoothstep(self._elapsed / (self.RAMP_CYCLES * self.period))
        if dt > 0.0:
            a = 1.0 - math.exp(-dt / self.cmd_tau)
            for j in range(3):
                self._filt[j] += a * (self._target[j] * ramp - self._filt[j])
        self._advance(dt)
        out: list[float] = []
        stance_xy: list[tuple[float, float]] = []
        for i in range(6):
            fx, fy, fz = self._foot_body(i)
            if i not in self._swing:
                stance_xy.append((fx, fy))
            yaw, hip, knee = self._leg_joints(i, fx, fy, fz)
            out.extend([math.degrees(yaw), math.degrees(hip),
                        math.degrees(knee)])
        m = support_margin(stance_xy, self.com_offset)
        self.last_support_margin = m
        if m < self.min_support_margin:
            self.min_support_margin = m
        return out


# ---------------------------------------------------------------------------
# Grid evaluation over planar commands (kinematic; no sim required).
def evaluate_command(vx: float, vy: float, wz: float, *,
                     gait_obj: SE2FootGait | None = None,
                     gait: str = "tetrapod", gait_kw: dict | None = None,
                     hip_deg: float = 20.0, knee_deg: float = 80.0,
                     dt: float = 0.05, warmup_cycles: float = 2.0,
                     measure_cycles: float = 1.0,
                     margin_min: float = 0.010) -> dict:
    """Run one command kinematically and report feasibility metrics.

    Pass ``gait_obj`` to reuse one instance across a grid (geometry and
    workspace radii are then computed once); it is reset per command.
    Metrics cover one full steady-state cycle after ``warmup_cycles``.
    """
    g = gait_obj
    if g is None:
        g = SE2FootGait(gait=gait, **(gait_kw or {}))
        g.sync_plant_stance(hip_deg, knee_deg)
    g.reset_phase()
    g.set_velocity(vx=vx, vy=vy, omega=wz)
    n_warm = max(1, int(round(warmup_cycles * g.period / dt)))
    n_meas = max(1, int(round(measure_cycles * g.period / dt)))
    t = 0.0
    q = g.desired_deg(t)
    for _ in range(n_warm):
        t += dt
        q = g.desired_deg(t)
    g.ik_failures = 0
    g.limit_clips = 0
    g.min_support_margin = float("inf")
    p0 = g.commanded_pose()
    prev = q
    max_rate = 0.0
    max_exc = 0.0
    min_stance = 6
    for _ in range(n_meas):
        t += dt
        q = g.desired_deg(t)
        max_rate = max(max_rate,
                       max(abs(a - b) for a, b in zip(q, prev)) / dt)
        prev = q
        stance = g.stance_legs()
        min_stance = min(min_stance, len(stance))
        for i in stance:
            fx, fy, _fz = g._foot_body(i)
            nx, ny = g.neutral_body[i]
            if g.workspace_r[i] > 1e-9:
                max_exc = max(max_exc, math.hypot(fx - nx, fy - ny)
                              / g.workspace_r[i])
    p1 = g.commanded_pose()
    c, s = math.cos(p0[2]), math.sin(p0[2])
    rx, ry = p1[0] - p0[0], p1[1] - p0[1]
    lx, ly, lth = se2_log(c * rx + s * ry, -s * rx + c * ry, p1[2] - p0[2])
    t_meas = n_meas * dt
    feasible = (g.ik_failures == 0 and g.limit_clips == 0
                and g.min_support_margin >= margin_min)
    return {
        "cmd": f"vx{vx:+.3f}_vy{vy:+.3f}_wz{wz:+.2f}",
        "vx": vx, "vy": vy, "wz": wz,
        "scale": round(g.last_command_scale, 4),
        "vx_ach": round(lx / t_meas, 4),
        "vy_ach": round(ly / t_meas, 4),
        "wz_ach": round(lth / t_meas, 4),
        "min_margin_m": round(g.min_support_margin, 4),
        "max_rate_dps": round(max_rate, 2),
        "max_excursion": round(max_exc, 3),
        "min_stance_feet": min_stance,
        "ik_failures": g.ik_failures,
        "limit_clips": g.limit_clips,
        "feasible": feasible,
    }


def evaluate_grid(*, gait: str = "tetrapod", gait_kw: dict | None = None,
                  hip_deg: float = 20.0, knee_deg: float = 80.0,
                  vx_max: float = SE2FootGait.MAX_VX,
                  vy_max: float = SE2FootGait.MAX_VY,
                  wz_max: float = SE2FootGait.MAX_OMEGA,
                  n: int = 5, dt: float = 0.05,
                  margin_min: float = 0.010,
                  verbose: bool = False) -> list[dict]:
    """Evaluate every command on the n x n x n planar grid."""
    g = SE2FootGait(gait=gait, **(gait_kw or {}))
    g.sync_plant_stance(hip_deg, knee_deg)

    def axis(m: float) -> list[float]:
        if n <= 1 or m <= 0.0:
            return [0.0]
        return [-m + 2.0 * m * j / (n - 1) for j in range(n)]

    rows: list[dict] = []
    for vx in axis(vx_max):
        for vy in axis(vy_max):
            for wz in axis(wz_max):
                row = evaluate_command(vx, vy, wz, gait_obj=g, dt=dt,
                                       margin_min=margin_min)
                rows.append(row)
                if verbose:
                    print(f"{row['cmd']:28s} scale={row['scale']:6.3f} "
                          f"margin={row['min_margin_m'] * 1000:6.1f}mm "
                          f"rate={row['max_rate_dps']:6.1f}dps "
                          f"excur={row['max_excursion']:5.2f} "
                          f"stance>={row['min_stance_feet']} "
                          f"{'ok' if row['feasible'] else 'INFEASIBLE'}",
                          flush=True)
    return rows


def main(argv: list[str] | None = None) -> int:
    import argparse
    import json
    from pathlib import Path

    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gait", default="tetrapod",
                    choices=("tetrapod", "wave"))
    ap.add_argument("--period", type=float, default=None)
    ap.add_argument("--swing-frac", type=float, default=None)
    ap.add_argument("--lift-mm", type=float, default=None)
    ap.add_argument("--hip", type=float, default=20.0,
                    help="plant hip deg (hardware default 20)")
    ap.add_argument("--knee", type=float, default=80.0,
                    help="plant knee deg (hardware default 80)")
    ap.add_argument("--cmd", action="append", default=None,
                    metavar="VX,VY,WZ",
                    help="evaluate exact triples instead of the grid")
    ap.add_argument("--n", type=int, default=5,
                    help="grid points per axis")
    ap.add_argument("--vx-max", type=float, default=SE2FootGait.MAX_VX)
    ap.add_argument("--vy-max", type=float, default=SE2FootGait.MAX_VY)
    ap.add_argument("--wz-max", type=float, default=SE2FootGait.MAX_OMEGA)
    ap.add_argument("--dt", type=float, default=0.05,
                    help="kinematic sample step (s)")
    ap.add_argument("--margin-min", type=float, default=0.010,
                    help="support margin floor for 'feasible' (m)")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args(argv)

    gait_kw: dict = {}
    if args.period is not None:
        gait_kw["period"] = args.period
    if args.swing_frac is not None:
        gait_kw["swing_frac"] = args.swing_frac
    if args.lift_mm is not None:
        gait_kw["lift"] = args.lift_mm * 1e-3

    common = dict(gait=args.gait, gait_kw=gait_kw, hip_deg=args.hip,
                  knee_deg=args.knee, dt=args.dt,
                  margin_min=args.margin_min)
    if args.cmd:
        g = SE2FootGait(gait=args.gait, **gait_kw)
        g.sync_plant_stance(args.hip, args.knee)
        print(f"workspace radii (mm): "
              f"{[round(r * 1000, 1) for r in g.workspace_r]}")
        rows = []
        for spec in args.cmd:
            vx, vy, wz = (float(x) for x in spec.split(","))
            row = evaluate_command(vx, vy, wz, gait_obj=g, dt=args.dt,
                                   margin_min=args.margin_min)
            rows.append(row)
            print(json.dumps(row, indent=1))
    else:
        rows = evaluate_grid(n=args.n, vx_max=args.vx_max,
                             vy_max=args.vy_max, wz_max=args.wz_max,
                             verbose=True, **common)
        n_ok = sum(r["feasible"] for r in rows)
        unscaled = sum(r["scale"] >= 0.999 for r in rows)
        print(f"\n{len(rows)} commands  feasible {n_ok}/{len(rows)}  "
              f"unscaled {unscaled}  "
              f"min margin {min(r['min_margin_m'] for r in rows) * 1000:.1f}mm"
              f"  max rate {max(r['max_rate_dps'] for r in rows):.1f} deg/s")

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(
            {"gait": args.gait, "gait_kw": gait_kw,
             "hip_deg": args.hip, "knee_deg": args.knee,
             "dt": args.dt, "margin_min": args.margin_min,
             "rows": rows}, indent=1))
        print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
