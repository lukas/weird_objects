"""Safe "go to zero": plan a collision-aware path to logical 0° and
execute it with limp-on-anomaly monitoring.

Operator ask (2026-08-10): a routine that works out how to get the legs
to zero (belly down, legs straight out) without banging into the ground
or each other, errors out if that is genuinely impossible, and goes
LIMP the moment any servo reports it is not turning or is fighting an
unexpected force.

Two halves:

PLANNER — ``plan_safe_zero(present)`` (pure stdlib math, testable on a
laptop with no hardware).

  1. "straighten"  hips → 0°, knees → a small lift angle that keeps the
     feet ~``LIFT_CLEAR_MM`` above the belly-down ground plane; yaws
     held where they are.

     From belly-down / tangled poses where the hip-frame floor model is
     meaningful, ``plan_ik_pose_transition`` makes the decision:
       a. if the direct joint-space path keeps every foot out of the
          ground and adjacent legs clear, take that one direct air move;
       b. otherwise try to lift/swing/place tripods, then individual
          legs, using per-leg IK so a foot that would touch the ground
          steps through the target instead of being dragged;
       c. if neither direct nor stepped geometry is provably clear,
          report failure to the caller.

     From a true STAND (feet well below the belly-down ground plane) the
     floor plane is not valid in the hip frame. Normal standing lowers
     are routed by the web/backend through STEP-down before safe-zero is
     considered. Legacy/offline callers still get a force-monitored
     straighten blend rather than a geometry claim we cannot justify.
  2. "center yaws" all yaws → 0° with the feet geometrically clear of
     the ground. Tries all-at-once; if the top-view leg segments would
     cross, falls back to one-leg-at-a-time (largest |yaw| first); if
     even that crosses, returns an ERROR (physically blocked — operator
     must reposition by hand).
  3. "extend"      knees → 0°. Feet rise from lift height to the zero
     plane (legs straight out never touch ground belly-down: the hip
     pivot sits ~40 mm up).

  Every stage's interpolated path is checked for adjacent-leg segment
  clearance; belly-phase stages also for ground clearance. Encoders
  outside the axis limits by > ``LIMIT_SLOP_DEG`` mean the logical zero
  frame is untrustworthy → refuse and tell the operator to
  ``/api/set_zero`` (geometry math on a wrong zero is meaningless).

EXECUTOR — ``run_safe_zero(bus, stages)``. One eased SyncWrite per
stage (servo-side trapezoid; host streaming buzzes), then a feedback
sweep loop (bulk ``read_all_feedback`` = 1 MCU round-trip). LIMPS ALL
SERVOS immediately on any of:

  * stall-fight: |I| over the limit while the joint is not moving and
    still far from target, two consecutive sweeps (house semantics,
    same as standup);
  * hard current cap (any single reading);
  * sustained high load while not moving;
  * "not turning": a commanded joint > 10° from target makes < 2° of
    progress over 2 s (quiet stall — servos give up under the torque
    limit without a current spike; measured on the blend standup);
  * a servo that stops answering feedback, or over-temp.

Operator abort (Stop button) HOLDS pose instead of limping — house
pattern, so a mid-descent abort doesn't drop the body.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

from feetech_bus import AXIS_LIMITS_DEG, N_JOINTS, joint_to_servo_id
from tripod_gait import (CHASSIS_FLAT_TO_FLAT_MM, COXA_MM, FEMUR_MM,
                         TIBIA_MM)

LEG_RADIAL_MM = CHASSIS_FLAT_TO_FLAT_MM / 2.0
AXIS_NAMES = ("yaw", "hip", "knee")

# Belly-down ground plane, z relative to the hip pivot (negative = below).
# Measured by geometry_plant contact search (foot_z_contact_mm ≈ -43);
# overridden from plant_pose.json when available.
BELLY_GROUND_Z_MM = -40.0
LIFT_CLEAR_MM = 22.0        # transit foot height above the ground plane
GROUND_TOL_MM = 3.0         # slack on the geometric ground check
LEG_CLEAR_MM = 30.0         # min top-view distance between adjacent legs
FOOT_SLIP_TOL_MM = 4.0      # max horizontal move while a foot is near ground
LIMIT_SLOP_DEG = 20.0       # encoder past limit+slop → zero frame suspect
DONE_TOL_DEG = 4.0
PATH_SAMPLES = 9

# Low-drag descent (standing starts only).
STAND_DETECT_MM = 25.0      # lowest foot this far below belly plane = stand
KNEE_FOLD_MAX_DEG = 135.0   # crouch fold ceiling (margin off the 150° stop)
KNEE_CONTACT_MAX_DEG = 130.0  # deepest fold allowed at belly touchdown
SKIM_CLEAR_MM = 6.0         # unloaded unfold sweeps this far over the floor
SLIDE_PRESS_TOL_MM = 6.0    # blend may push a foot this far below its line
SLIDE_DEV_TOL_MM = 8.0      # blend may slide a foot this far off its radius
DESCENT_RATE_DPS = 12.0
_REFINE_DEPTH = 2           # waypoint bisection depth before giving up

# Executor trip thresholds.
STALL_CURRENT_A = 2.5       # air / geometric stages
DRAG_CURRENT_A = 3.0        # descent stage (weight transfer is honest work)
HARD_CAP_A = 3.5
LOAD_MAX_PCT = 70.0
DRAG_LOAD_MAX_PCT = 85.0
SLOW_DPS = 8.0              # below this the joint counts as "not moving"
NO_PROGRESS_S = 2.0
NO_PROGRESS_DEG = 2.0
NO_PROGRESS_MIN_ERR = 10.0
TEMP_MAX_C = 63
SETTLE_DEG = 3.5
FB_MISS_LIMIT = 3
# Operator rule (08-11): surprise force can always be a mechanical jam
# or a WRONG LOGICAL ZERO (targets are absolute angles in that frame).
# Every force trip already limps; the error must also say so, or the
# next thing anyone tries is the same move again.
FORCE_HINT = " — possible mechanical jam or wrong zero (re-do set_zero)"


def joint_name(j: int) -> str:
    return f"L{j // 3} {AXIS_NAMES[j % 3]}"


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------

def foot_z_mm(hip_deg: float, knee_deg: float) -> float:
    """Foot Z relative to the hip pivot (mm); negative = below."""
    p = math.radians(hip_deg)
    k = math.radians(knee_deg)
    return -FEMUR_MM * math.sin(p) - TIBIA_MM * math.sin(k)


def knee_for_foot_z(hip_deg: float, z_mm: float) -> float | None:
    """Knee (deg) putting the foot at ``z_mm`` for a fixed hip; None if
    unreachable within the knee axis limits."""
    lo, hi = AXIS_LIMITS_DEG[2]
    p = math.radians(hip_deg)
    num = -(z_mm + FEMUR_MM * math.sin(p)) / TIBIA_MM
    if abs(num) > 1.0:
        return None
    a = math.asin(max(-1.0, min(1.0, num)))
    for cand in (a, math.pi - a):
        knee = math.degrees(cand)
        if lo <= knee <= hi:
            return float(knee)
    return None


def foot_r_mm(hip_deg: float, knee_deg: float) -> float:
    """Foot radial distance from the hip pivot in the leg plane (mm)."""
    return (FEMUR_MM * math.cos(math.radians(hip_deg))
            + TIBIA_MM * math.cos(math.radians(knee_deg)))


def leg_reach_mm(knee_deg: float) -> float:
    """Hip-pivot→foot distance at a given knee fold (hip-independent)."""
    return math.sqrt(FEMUR_MM ** 2 + TIBIA_MM ** 2
                     + 2.0 * FEMUR_MM * TIBIA_MM
                     * math.cos(math.radians(knee_deg)))


def ik_hip_knee(r_mm: float, z_mm: float) -> tuple[float, float] | None:
    """(hip, knee) placing the foot at ``(r, z)`` mm from the hip pivot.

    Absolute-tibia convention: hip is femur angle, knee is tibia angle in
    the same leg plane. None when out of reach or outside axis limits.
    """
    w = -float(z_mm)
    u = float(r_mm)
    L = math.hypot(u, w)
    if L < 1e-6 or L > FEMUR_MM + TIBIA_MM - 1e-6:
        return None
    if L < abs(FEMUR_MM - TIBIA_MM) + 1e-6:
        return None
    hip_lo, hip_hi = AXIS_LIMITS_DEG[1]
    knee_lo, knee_hi = AXIS_LIMITS_DEG[2]
    gamma = math.atan2(w, u)
    ca = (L * L + FEMUR_MM ** 2 - TIBIA_MM ** 2) / (2.0 * L * FEMUR_MM)
    ca = max(-1.0, min(1.0, ca))
    alpha = math.acos(ca)
    candidates = []
    for hip_r in (gamma - alpha, gamma + alpha):
        knee_r = math.atan2(w - FEMUR_MM * math.sin(hip_r),
                            u - FEMUR_MM * math.cos(hip_r))
        hip = math.degrees(hip_r)
        knee = math.degrees(knee_r)
        if hip_lo <= hip <= hip_hi and knee_lo <= knee <= knee_hi:
            score = abs(knee - hip)
            if knee < hip:
                score += 100.0
            candidates.append((score, hip, knee))
    if not candidates:
        return None
    _score, hip, knee = min(candidates, key=lambda row: row[0])
    return float(hip), float(knee)


def ik_leg_angles(yaw_deg: float, r_mm: float,
                  z_mm: float) -> tuple[float, float, float] | None:
    """Full per-leg IK: ``(yaw, hip, knee)`` for a leg-plane foot target.

    ``r_mm``/``z_mm`` are measured from the hip pivot in the leg's yawed
    plane, using the same absolute-tibia convention as the rest of this
    module. This is the public helper to use when a planner knows a foot
    should be at a geometric point instead of hand-editing hip/knee angles.
    """
    yaw_lo, yaw_hi = AXIS_LIMITS_DEG[0]
    yaw = float(yaw_deg)
    if yaw < yaw_lo or yaw > yaw_hi:
        return None
    hk = ik_hip_knee(r_mm, z_mm)
    if hk is None:
        return None
    return yaw, hk[0], hk[1]


def leg_segment_2d(leg: int, yaw_deg: float, hip_deg: float,
                   knee_deg: float) -> tuple[tuple[float, float],
                                             tuple[float, float]]:
    """Top-view (x, y) mm of the leg: yaw-axis root → foot tip."""
    az = math.radians((leg + 0.5) * 60.0)
    root = (LEG_RADIAL_MM * math.cos(az), LEG_RADIAL_MM * math.sin(az))
    ang = az + math.radians(yaw_deg)
    u = (COXA_MM + FEMUR_MM * math.cos(math.radians(hip_deg))
         + TIBIA_MM * math.cos(math.radians(knee_deg)))
    return root, (root[0] + u * math.cos(ang), root[1] + u * math.sin(ang))


def _pt_seg_dist(p, a, b) -> float:
    ax, ay = a
    vx, vy = b[0] - ax, b[1] - ay
    L2 = vx * vx + vy * vy
    if L2 <= 1e-12:
        return math.hypot(p[0] - ax, p[1] - ay)
    t = max(0.0, min(1.0, ((p[0] - ax) * vx + (p[1] - ay) * vy) / L2))
    return math.hypot(p[0] - (ax + t * vx), p[1] - (ay + t * vy))


def _segs_intersect(a, b, c, d) -> bool:
    def orient(p, q, r):
        v = ((q[0] - p[0]) * (r[1] - p[1])
             - (q[1] - p[1]) * (r[0] - p[0]))
        return 0 if abs(v) < 1e-9 else (1 if v > 0 else -1)
    o1, o2 = orient(a, b, c), orient(a, b, d)
    o3, o4 = orient(c, d, a), orient(c, d, b)
    return o1 != o2 and o3 != o4 and 0 not in (o1, o2, o3, o4)


def seg_dist_2d(a, b, c, d) -> float:
    """Min distance between 2D segments ab and cd (0 if they cross)."""
    if _segs_intersect(a, b, c, d):
        return 0.0
    return min(_pt_seg_dist(a, c, d), _pt_seg_dist(b, c, d),
               _pt_seg_dist(c, a, b), _pt_seg_dist(d, a, b))


def _adjacent_dists(q: list[float]) -> dict[tuple[int, int], float]:
    """Top-view clearance between each adjacent leg pair."""
    segs = [leg_segment_2d(leg, q[leg * 3], q[leg * 3 + 1], q[leg * 3 + 2])
            for leg in range(6)]
    out: dict[tuple[int, int], float] = {}
    for i in range(6):
        j = (i + 1) % 6
        out[(i, j)] = seg_dist_2d(*segs[i], *segs[j])
    return out


def _foot_xy(q: list[float], leg: int) -> tuple[float, float]:
    """Top-view foot-tip position for slip checks."""
    return leg_segment_2d(
        leg, q[leg * 3], q[leg * 3 + 1], q[leg * 3 + 2])[1]


def _lerp(q0: list[float], q1: list[float], t: float) -> list[float]:
    return [a + (b - a) * t for a, b in zip(q0, q1)]


def _max_delta(q0: list[float], q1: list[float]) -> float:
    return max(abs(b - a) for a, b in zip(q0, q1))


def _path_violation(q0: list[float], q1: list[float], *,
                    ground_z_mm: float | None = None,
                    clear_mm: float = LIFT_CLEAR_MM,
                    check_slip: bool = True) -> str | None:
    """Check the linear path q0→q1. Returns a description or None.

    Collision rule tolerates a pair that STARTS inside the margin as
    long as the move does not bring it closer (so separating crossed
    legs is always allowed).
    """
    base = _adjacent_dists(q0)
    prev_z = [foot_z_mm(q0[leg * 3 + 1], q0[leg * 3 + 2])
              for leg in range(6)]
    prev_xy = [_foot_xy(q0, leg) for leg in range(6)]
    for k in range(1, PATH_SAMPLES + 1):
        q = _lerp(q0, q1, k / PATH_SAMPLES)
        if ground_z_mm is not None:
            for leg in range(6):
                z = foot_z_mm(q[leg * 3 + 1], q[leg * 3 + 2])
                if z < ground_z_mm + clear_mm - GROUND_TOL_MM:
                    return (f"L{leg} foot would drop to "
                            f"{z - ground_z_mm:.0f} mm over ground "
                            f"(need ≥{clear_mm - GROUND_TOL_MM:.0f} mm)")
                if check_slip:
                    contact = z <= ground_z_mm + GROUND_TOL_MM
                    prev_contact = prev_z[leg] <= ground_z_mm + GROUND_TOL_MM
                    if contact and prev_contact:
                        xy = _foot_xy(q, leg)
                        slip = math.hypot(xy[0] - prev_xy[leg][0],
                                          xy[1] - prev_xy[leg][1])
                        if slip > FOOT_SLIP_TOL_MM:
                            return (f"L{leg} foot would slide "
                                    f"{slip:.0f} mm while near ground")
        for (i, j), d in _adjacent_dists(q).items():
            if d < LEG_CLEAR_MM and d < base[(i, j)] - 1.0:
                return (f"legs L{i}/L{j} would close to {d:.0f} mm "
                        f"(need ≥{LEG_CLEAR_MM:.0f} mm)")
        if ground_z_mm is not None:
            prev_z = [foot_z_mm(q[leg * 3 + 1], q[leg * 3 + 2])
                      for leg in range(6)]
            prev_xy = [_foot_xy(q, leg) for leg in range(6)]
    return None


# ---------------------------------------------------------------------------
# Planner
# ---------------------------------------------------------------------------

def belly_ground_z_mm() -> float:
    """Ground plane from the captured plant's contact search, else default."""
    try:
        from feetech_bus import load_plant_pose
        p = load_plant_pose()
        if p.get("path"):
            raw = json.loads(Path(p["path"]).read_text())
            z = raw.get("foot_z_contact_mm")
            if isinstance(z, (int, float)) and -90.0 <= z <= -15.0:
                return float(z)
    except Exception:
        pass
    return BELLY_GROUND_Z_MM


def _stage(label: str, goal: list[float], seconds: float,
           drag_ok: bool) -> dict:
    return {"label": label, "goal": [round(v, 3) for v in goal],
            "seconds": round(seconds, 2), "drag_ok": drag_ok}


# ---------------------------------------------------------------------------
# Low-drag descent (standing starts)
# ---------------------------------------------------------------------------

def _descent_pose(yaw_now: list[float],
                  feet: list[tuple[float, float]]) -> list[float] | None:
    """18-joint pose putting each foot at its (r, z); None if any leg
    has no in-limit IK solution."""
    q: list[float] = []
    for leg in range(6):
        hk = ik_hip_knee(*feet[leg])
        if hk is None:
            return None
        q.extend([yaw_now[leg], hk[0], hk[1]])
    return q


def _foot_rz(q: list[float], leg: int) -> tuple[float, float]:
    hip, knee = q[leg * 3 + 1], q[leg * 3 + 2]
    return foot_r_mm(hip, knee), foot_z_mm(hip, knee)


def _reachable_r_at_z(r_mm: float, z_mm: float) -> float:
    """Nudge an air target inside reach while preserving direction.

    Near full extension, "lift this foot straight up at the same radius" can
    be a millimeter outside the two-link workspace. The planner may retract an
    unloaded foot slightly while it is being lifted; it must not do this for a
    loaded/direct path, which is why this helper is used only by step stages.
    """
    max_reach = FEMUR_MM + TIBIA_MM - 2.0
    z = float(z_mm)
    if math.hypot(float(r_mm), -z) <= max_reach:
        return float(r_mm)
    lim = math.sqrt(max(0.0, max_reach ** 2 - z ** 2))
    return math.copysign(lim, float(r_mm) if abs(r_mm) > 1e-9 else 1.0)


def _set_leg_ik(q: list[float], leg: int, yaw_deg: float,
                r_mm: float, z_mm: float) -> bool:
    angles = ik_leg_angles(yaw_deg, _reachable_r_at_z(r_mm, z_mm), z_mm)
    if angles is None:
        return False
    q[leg * 3:leg * 3 + 3] = [angles[0], angles[1], angles[2]]
    return True


def _transition_stage(label: str, q0: list[float], q1: list[float],
                      *, min_s: float = 1.0, max_s: float = 6.0,
                      rate_dps: float = 24.0) -> dict | None:
    d = _max_delta(q0, q1)
    if d <= 1.0:
        return None
    return _stage(label, q1, min(max_s, max(min_s, d / rate_dps)), False)


def _step_group_to_goal(q0: list[float], goal: list[float],
                        legs: tuple[int, ...], *, label: str,
                        ground_z_mm: float,
                        lift_clear_mm: float) -> list[dict] | None:
    q = list(q0)
    stages: list[dict] = []
    leg_label = "/".join(f"L{leg}" for leg in legs)

    def _append(next_q: list[float], suffix: str, *,
                min_s: float = 1.0, max_s: float = 6.0,
                rate_dps: float = 24.0) -> bool:
        v = _path_violation(q, next_q, ground_z_mm=ground_z_mm,
                            clear_mm=0.0)
        if v:
            return False
        st = _transition_stage(f"{label}: step {leg_label} {suffix}",
                               q, next_q, min_s=min_s,
                               max_s=max_s, rate_dps=rate_dps)
        if st is not None:
            stages.append(st)
        q[:] = next_q
        return True

    lift_q = list(q)
    clear_z_by_leg: dict[int, float] = {}
    for leg in legs:
        cur_r, cur_z = _foot_rz(q, leg)
        goal_r, goal_z = _foot_rz(goal, leg)
        clear_z = max(cur_z, goal_z, ground_z_mm + lift_clear_mm)
        clear_z_by_leg[leg] = clear_z
        if not _set_leg_ik(lift_q, leg, q[leg * 3], cur_r, clear_z):
            return None
    if not _append(lift_q, "lift", min_s=0.8, max_s=4.0,
                   rate_dps=28.0):
        return None

    swing_q = list(q)
    for leg in legs:
        goal_r, _goal_z = _foot_rz(goal, leg)
        if not _set_leg_ik(swing_q, leg, goal[leg * 3],
                           goal_r, clear_z_by_leg[leg]):
            return None
    if not _append(swing_q, "swing", min_s=0.8, max_s=5.0,
                   rate_dps=24.0):
        return None

    place_q = list(q)
    for leg in legs:
        place_q[leg * 3:leg * 3 + 3] = goal[leg * 3:leg * 3 + 3]
    if not _append(place_q, "place", min_s=0.8, max_s=4.0,
                   rate_dps=28.0):
        return None
    return stages


def _step_groups_to_goal(present: list[float], goal: list[float],
                         groups: list[tuple[int, ...]], *,
                         label: str, ground_z_mm: float,
                         lift_clear_mm: float) -> list[dict] | None:
    q = list(present)
    stages: list[dict] = []
    for legs in groups:
        part = _step_group_to_goal(
            q, goal, legs, label=label, ground_z_mm=ground_z_mm,
            lift_clear_mm=lift_clear_mm)
        if part is None:
            return None
        stages.extend(part)
        if part:
            q = [float(v) for v in part[-1]["goal"]]
    if _max_delta(q, goal) > 1.5:
        return None
    return stages


def plan_ik_pose_transition(present: list[float], goal: list[float], *,
                            label: str,
                            ground_z_mm: float = BELLY_GROUND_Z_MM,
                            lift_clear_mm: float = LIFT_CLEAR_MM) -> dict:
    """Plan a transition to ``goal`` using geometry before brute force.

    Decision order:
    1. If the straight joint-space path keeps feet off the ground and
       adjacent legs clear, use one direct stage.
    2. If the straight path would touch the ground or close leg segments,
       step unloaded feet through the target: tripods first, then one leg at
       a time if tripod stepping cannot be proven clear.

    This function is pure geometry. It never touches hardware and it never
    emits intentional sliding stages; callers that accept dragging must keep
    that fallback explicit.
    """
    present = [float(v) for v in present]
    goal = [float(v) for v in goal]
    if len(present) != N_JOINTS or len(goal) != N_JOINTS:
        return {"ok": False, "error": "transition needs 18-joint poses"}
    d = _max_delta(present, goal)
    if d <= 1.0:
        return {"ok": True, "stages": [], "strategy": "already_at_goal"}

    direct_v = _path_violation(present, goal, ground_z_mm=ground_z_mm,
                               clear_mm=0.0)
    if direct_v is None:
        return {
            "ok": True,
            "strategy": "direct_clear",
            "stages": [_stage(f"{label} (direct clear path)", goal,
                              min(8.0, max(1.2, d / 18.0)), False)],
        }

    tripod = _step_groups_to_goal(
        present, goal, [(0, 2, 4), (1, 3, 5)], label=label,
        ground_z_mm=ground_z_mm, lift_clear_mm=lift_clear_mm)
    if tripod is not None:
        return {"ok": True, "strategy": "step_tripods",
                "direct_blocked_by": direct_v, "stages": tripod}

    singles = _step_groups_to_goal(
        present, goal, [(0,), (1,), (2,), (3,), (4,), (5,)], label=label,
        ground_z_mm=ground_z_mm, lift_clear_mm=lift_clear_mm)
    if singles is not None:
        return {"ok": True, "strategy": "step_single_legs",
                "direct_blocked_by": direct_v, "stages": singles}

    return {"ok": False,
            "error": (f"direct path blocked ({direct_v}) and no "
                      "collision-free stepping order reached the target")}


def _seg_foot_error(qa: list[float], qb: list[float],
                    fa: list[tuple[float, float]],
                    fb: list[tuple[float, float]], *,
                    floor_z_mm: float | None) -> tuple[float, float, float]:
    """How badly the joint-space blend qa→qb strays from the straight
    per-leg foot line fa→fb.

    Returns (press, r_dev, floor_hit) in mm: press = worst dip BELOW
    the foot line (pushes into whatever the foot rests on); r_dev =
    worst radial wander (unplanned slide); floor_hit = worst intrusion
    of a foot or knee joint below a fixed floor (belly-down segments
    only; the floor plane is meaningless in the hip frame while the
    body is still descending).
    """
    press = r_dev = floor_hit = 0.0
    for s in range(1, PATH_SAMPLES):
        t = s / PATH_SAMPLES
        q = _lerp(qa, qb, t)
        for leg in range(6):
            hip, knee = q[leg * 3 + 1], q[leg * 3 + 2]
            r, z = foot_r_mm(hip, knee), foot_z_mm(hip, knee)
            rl = fa[leg][0] + (fb[leg][0] - fa[leg][0]) * t
            zl = fa[leg][1] + (fb[leg][1] - fa[leg][1]) * t
            press = max(press, zl - z)
            r_dev = max(r_dev, abs(r - rl))
            if floor_z_mm is not None:
                floor_hit = max(
                    floor_hit, floor_z_mm - GROUND_TOL_MM - z)
                knee_z = -FEMUR_MM * math.sin(math.radians(hip))
                floor_hit = max(floor_hit, floor_z_mm - knee_z)
    return press, r_dev, floor_hit


def _refine_pair(a: dict, b: dict, yaw_now: list[float],
                 ground_z_mm: float, depth: int) -> list[dict] | None:
    """Waypoints (excluding ``a``) so every blend tracks its foot line;
    bisects through IK up to ``depth`` times, None if it can't."""
    floor = ground_z_mm if a.get("grounded") else None
    press, r_dev, floor_hit = _seg_foot_error(
        a["q"], b["q"], a["feet"], b["feet"], floor_z_mm=floor)
    if (press <= SLIDE_PRESS_TOL_MM and r_dev <= SLIDE_DEV_TOL_MM
            and floor_hit <= 0.0):
        return [b]
    if depth <= 0:
        return None
    mid_feet = [((ra + rb) / 2.0, (za + zb) / 2.0)
                for (ra, za), (rb, zb) in zip(a["feet"], b["feet"])]
    qm = _descent_pose(yaw_now, mid_feet)
    if qm is None:
        return None
    mid = {"q": qm, "feet": mid_feet, "label": b["label"],
           "grounded": a.get("grounded")}
    left = _refine_pair(a, mid, yaw_now, ground_z_mm, depth - 1)
    if left is None:
        return None
    right = _refine_pair(mid, b, yaw_now, ground_z_mm, depth - 1)
    if right is None:
        return None
    return left + right


def _plan_descent(present: list[float], yaw_now: list[float],
                  ground_z_mm: float, knee_lift: float,
                  z_lift: float) -> dict:
    """Drag-minimal stand→belly descent ending exactly at the lift pose.

    ``{"ok": True, "stages": [...], ...metrics}`` or
    ``{"ok": False, "why": ...}`` (caller falls back to the legacy
    single-blend straighten).
    """
    r0, z0 = [], []
    for leg in range(6):
        hip, knee = present[leg * 3 + 1], present[leg * 3 + 2]
        r0.append(foot_r_mm(hip, knee))
        z0.append(foot_z_mm(hip, knee))
    if min(z0) > ground_z_mm - STAND_DETECT_MM:
        return {"ok": False, "why": "not standing"}
    r_lift = foot_r_mm(0.0, knee_lift)

    points = [{"q": list(present), "feet": list(zip(r0, z0)),
               "grounded": False}]

    def _add(label: str, feet: list[tuple[float, float]],
             grounded: bool, q: list[float] | None = None) -> bool:
        q = q if q is not None else _descent_pose(yaw_now, feet)
        if q is None:
            return False
        if _max_delta(points[-1]["q"], q) > 1.5:
            points.append({"q": q, "feet": feet, "label": label,
                           "grounded": grounded})
        elif grounded:
            points[-1]["grounded"] = True
        return True

    ok = _add("lower body (feet planted)",
              [(r0[leg], ground_z_mm) for leg in range(6)], True)
    z_skim = ground_z_mm + SKIM_CLEAR_MM
    ok = ok and _add(
        "skim feet to lift radius (unloaded)",
        [((r0[leg] + r_lift) / 2.0, z_skim) for leg in range(6)], True)
    q_lift_goal = []
    for leg in range(6):
        q_lift_goal.extend([yaw_now[leg], 0.0, knee_lift])
    ok = ok and _add("feet to lift height",
                     [(r_lift, z_lift)] * 6, True, q=q_lift_goal)
    if not ok or len(points) < 2:
        return {"ok": False, "why": "no in-limit leg path for this pose"}

    refined = [points[0]]
    for a, b in zip(points, points[1:]):
        seg = _refine_pair(a, b, yaw_now, ground_z_mm, _REFINE_DEPTH)
        if seg is None:
            return {"ok": False,
                    "why": "blend cannot track the planned foot path"}
        refined.extend(seg)

    stages = []
    for a, b in zip(refined, refined[1:]):
        d = _max_delta(a["q"], b["q"])
        stages.append(_stage(b["label"], b["q"],
                             min(12.0, max(2.0, d / DESCENT_RATE_DPS)),
                             not a.get("grounded")))
    legacy = max(abs(r_lift - r0[leg]) for leg in range(6))
    return {"ok": True, "stages": stages,
            "loaded_slide_mm": 0.0,
            "legacy_slide_mm": round(legacy, 1),
            "crouch_height_mm": round(-ground_z_mm, 1)}


def plan_safe_zero(present: list[float], *,
                   ground_z_mm: float = BELLY_GROUND_Z_MM,
                   lift_clear_mm: float = LIFT_CLEAR_MM,
                   done_tol_deg: float = DONE_TOL_DEG) -> dict:
    """Plan staged waypoints from ``present`` (18 joint deg) to all-0°.

    Returns ``{"ok": True, "stages": [...]}`` or
    ``{"ok": False, "error": ...}`` when no safe path exists.
    Pure geometry — never touches hardware.
    """
    if (not isinstance(present, (list, tuple)) or len(present) != N_JOINTS
            or any(v is None or not math.isfinite(float(v))
                   for v in present)):
        return {"ok": False,
                "error": "need all 18 present joint angles to plan"}
    present = [float(v) for v in present]

    for j, v in enumerate(present):
        lo, hi = AXIS_LIMITS_DEG[j % 3]
        if v < lo - LIMIT_SLOP_DEG or v > hi + LIMIT_SLOP_DEG:
            return {
                "ok": False, "joint": j, "code": "suspect_zero",
                "error": (
                    f"{joint_name(j)} reads {v:+.1f}° — outside its "
                    f"{lo:.0f}..{hi:.0f}° range by more than "
                    f"{LIMIT_SLOP_DEG:.0f}°. The logical zero frame looks "
                    "wrong; hand-set the known pose and POST /api/set_zero "
                    "before any absolute move."),
            }

    if max(abs(v) for v in present) <= done_tol_deg:
        return {"ok": True, "stages": [], "already_at_zero": True,
                "ground_z_mm": ground_z_mm}

    z_lift = ground_z_mm + lift_clear_mm
    knee_lift = knee_for_foot_z(0.0, z_lift)
    if knee_lift is None:
        return {"ok": False,
                "error": (f"no knee angle reaches foot z {z_lift:.0f} mm "
                          "at hip 0° — ground model looks wrong")}

    yaw_lo, yaw_hi = AXIS_LIMITS_DEG[0]
    yaw_now = [min(yaw_hi, max(yaw_lo, present[leg * 3]))
               for leg in range(6)]

    q_lift, q_center = [], []
    for leg in range(6):
        q_lift.extend([yaw_now[leg], 0.0, knee_lift])
        q_center.extend([0.0, 0.0, knee_lift])
    q_zero = [0.0] * N_JOINTS

    stages: list[dict] = []
    notes: list[str] = []

    # Stage 1 — reach the lifted-yaw-current pose.
    #
    # If the belly-down ground model is valid, prefer the IK transition
    # helper: direct clear path first, then stepping unloaded feet by
    # tripods / single legs so we do not drag feet across the floor. If
    # the pose is truly standing, the ground plane in the hip frame is
    # not valid; the smart web/backend layer routes normal standing lower
    # to STEP-down before this planner, and this pure function keeps the
    # old force-monitored fallback for offline callers/tests.
    descent_used = None
    d1 = _max_delta(present, q_lift)
    if d1 > 1.0:
        lowest_z = min(foot_z_mm(present[leg * 3 + 1],
                                 present[leg * 3 + 2])
                       for leg in range(6))
        is_standing = lowest_z < ground_z_mm - STAND_DETECT_MM
        stage1_done = False
        if is_standing:
            desc = _plan_descent(present, yaw_now, ground_z_mm,
                                 knee_lift, z_lift)
            if desc["ok"]:
                prev = present
                for s in desc["stages"]:
                    v = _path_violation(prev, s["goal"], ground_z_mm=None)
                    if v:
                        desc = {"ok": False, "why": v}
                        break
                    prev = s["goal"]
            if desc["ok"]:
                stages.extend(desc["stages"])
                descent_used = {k: desc[k] for k in
                                ("loaded_slide_mm", "legacy_slide_mm",
                                 "crouch_height_mm")}
                notes.append(
                    f"low-drag descent: lower to belly plane "
                    f"{desc['crouch_height_mm']:.0f} mm height, loaded "
                    f"slide <= {desc['loaded_slide_mm']:.0f} mm "
                    f"(legacy blend ~= {desc['legacy_slide_mm']:.0f} mm)")
                stage1_done = True
            else:
                notes.append(f"low-drag descent unavailable "
                             f"({desc.get('why')}); using monitored "
                             "straighten blend")
                v = _path_violation(present, q_lift, ground_z_mm=None)
                if v:
                    return {"ok": False,
                            "error": (f"cannot straighten legs safely: {v}. "
                                      "Reposition the crossed legs by hand "
                                      "(limp) and retry.")}
                stages.append(_stage(
                    "straighten hips/knees (feet to lift height)",
                    q_lift, min(12.0, max(3.0, d1 / 12.0)), True))
                stage1_done = True
        if not stage1_done:
            trans = plan_ik_pose_transition(
                present, q_lift, label="straighten hips/knees",
                ground_z_mm=ground_z_mm, lift_clear_mm=lift_clear_mm)
            if trans["ok"]:
                stages.extend(trans["stages"])
                notes.append(f"IK transition to lift pose: "
                             f"{trans['strategy']}")
                stage1_done = True
            else:
                notes.append(f"IK transition unavailable "
                             f"({trans.get('error')}); using monitored "
                             "straighten blend")
                v = _path_violation(present, q_lift, ground_z_mm=None)
                if v:
                    return {"ok": False,
                            "error": (f"cannot straighten legs safely: {v}. "
                                      "Reposition the crossed legs by hand "
                                      "(limp) and retry.")}
                stages.append(_stage(
                    "straighten hips/knees (feet to lift height)",
                    q_lift, min(12.0, max(3.0, d1 / 12.0)), True))
                stage1_done = True

    # Stage 2 — yaws to center with the feet geometrically clear.
    d2 = _max_delta(q_lift, q_center)
    if d2 > 1.5:
        v = _path_violation(q_lift, q_center, ground_z_mm=ground_z_mm,
                            clear_mm=lift_clear_mm)
        if v is None:
            stages.append(_stage(
                "center all yaws", q_center,
                min(6.0, max(2.0, d2 / 18.0)), False))
        else:
            notes.append(f"simultaneous yaw sweep unsafe ({v}); "
                         "centering one leg at a time")
            q = list(q_lift)
            order = sorted(range(6), key=lambda l: -abs(yaw_now[l]))
            for leg in order:
                if abs(q[leg * 3]) <= 1.0:
                    continue
                q_next = list(q)
                q_next[leg * 3] = 0.0
                v2 = _path_violation(q, q_next, ground_z_mm=ground_z_mm,
                                     clear_mm=lift_clear_mm)
                if v2:
                    return {"ok": False,
                            "error": (f"no collision-free yaw order: {v2} "
                                      f"even moving L{leg} alone. Legs are "
                                      "physically tangled — limp and "
                                      "reposition by hand.")}
                stages.append(_stage(
                    f"center yaw L{leg}", q_next,
                    min(5.0, max(1.5, abs(q[leg * 3]) / 18.0)), False))
                q = q_next

    # Stage 3 — extend knees flat. Feet rise from lift height to the
    # zero plane, which sits above the belly-down ground.
    d3 = _max_delta(q_center, q_zero)
    if d3 > 1.0:
        v = _path_violation(q_center, q_zero, ground_z_mm=ground_z_mm,
                            clear_mm=0.0)
        if v:
            return {"ok": False,
                    "error": f"final extension blocked: {v}"}
        stages.append(_stage(
            "extend legs straight (zero)", q_zero,
            min(5.0, max(2.0, d3 / 15.0)), False))

    out = {
        "ok": True,
        "stages": stages,
        "notes": notes,
        "ground_z_mm": round(ground_z_mm, 1),
        "knee_lift_deg": round(knee_lift, 2),
        "total_s": round(sum(s["seconds"] for s in stages), 1),
    }
    if descent_used:
        out["descent"] = descent_used
    return out


# ---------------------------------------------------------------------------
# Executor
# ---------------------------------------------------------------------------

def run_safe_zero(bus, stages: list[dict], *,
                  abort_check=None,
                  on_progress=None,
                  torque_limit: int = 700) -> dict:
    """Execute planner stages with per-sweep anomaly monitoring.

    Any trip (stall-fight, hard current cap, sustained load, quiet
    no-progress stall, lost feedback, over-temp) → LIMP ALL SERVOS and
    return ``{"ok": False, "limp": True, ...}``. Operator abort → hold.
    """
    from inplace_demos import (_enable_torque, _glide_speed_acc, _hold_here,
                               _limp_all, _live_robot_ids, _read_pose,
                               _set_torque_limit, _write_pose)
    check = abort_check or (lambda: False)
    prog = on_progress or (lambda d: None)

    live = _live_robot_ids(bus)
    missing = [joint_to_servo_id(j) for j in range(N_JOINTS)
               if joint_to_servo_id(j) not in live]
    if missing:
        return {"ok": False,
                "error": (f"servo IDs {missing} not answering — safe zero "
                          "needs all 18 joints for its monitoring")}

    bulk = hasattr(bus, "read_all_feedback")

    def _sweep() -> dict[int, dict]:
        if bulk:
            try:
                return bus.read_all_feedback() or {}
            except Exception:
                return {}
        out: dict[int, dict] = {}
        for j in range(N_JOINTS):
            try:
                fb = bus.read_feedback(j)
            except Exception:
                fb = None
            if fb:
                out[j] = fb
        return out

    peak_a, peak_j = 0.0, None

    def _trip(reason: str, stage_label: str) -> dict:
        try:
            _limp_all(bus, live)
        except Exception:
            pass
        return {"ok": False, "limp": True, "stage": stage_label,
                "error": f"LIMP — {reason}",
                "peak_a": round(peak_a, 2), "peak_joint": peak_j}

    _set_torque_limit(bus, live, int(torque_limit))
    _enable_torque(bus, live)
    n = len(stages)
    try:
        for si, st in enumerate(stages):
            goal = [float(v) for v in st["goal"]]
            secs = float(st["seconds"])
            drag = bool(st.get("drag_ok"))
            label = str(st.get("label") or f"stage {si + 1}")
            cur_lim = DRAG_CURRENT_A if drag else STALL_CURRENT_A
            load_lim = DRAG_LOAD_MAX_PCT if drag else LOAD_MAX_PCT

            if check():
                _hold_here(bus, live)
                return {"ok": False, "aborted": True, "stage": label}
            start = _read_pose(bus, live)
            speed, acc = _glide_speed_acc(start, goal, live, secs)
            prog({"msg": f"safe_zero {si + 1}/{n}: {label}",
                  "stage": si + 1, "of": n})
            _write_pose(bus, goal, live, speed=speed, acc=acc)

            t0 = time.monotonic()
            timeout = secs * 1.8 + 2.5
            min_settle = max(0.4, secs * 0.5)
            fb_interval = 0.3 if bulk else 0.6
            last_fb = 0.0
            stall_prev: set[int] = set()
            load_prev: set[int] = set()
            temp_prev: set[int] = set()
            miss_count: dict[int, int] = {}
            sweep_misses = 0
            progress_ref: dict[int, tuple[float, float]] = {}
            worst_err = float("inf")
            settled = 0

            while True:
                if check():
                    _hold_here(bus, live)
                    return {"ok": False, "aborted": True, "stage": label}
                now = time.monotonic()
                if now - t0 > timeout:
                    if worst_err > 8.0:
                        return _trip(
                            f"timed out {worst_err:.0f}° short of target "
                            f"— joints not tracking", label)
                    break
                if now - last_fb >= fb_interval:
                    fb_map = _sweep()
                    last_fb = time.monotonic()
                    if not fb_map:
                        sweep_misses += 1
                        if sweep_misses >= FB_MISS_LIMIT:
                            return _trip("feedback lost (bus not "
                                         "answering)", label)
                        time.sleep(0.1)
                        continue
                    sweep_misses = 0

                    for j in range(N_JOINTS):
                        if j in fb_map:
                            miss_count[j] = 0
                        else:
                            miss_count[j] = miss_count.get(j, 0) + 1
                            if miss_count[j] >= FB_MISS_LIMIT:
                                return _trip(
                                    f"{joint_name(j)} stopped answering "
                                    "feedback", label)

                    errs = {j: abs(goal[j] - float(fb["deg"]))
                            for j, fb in fb_map.items()}
                    worst_err = max(errs.values(), default=0.0)

                    def _slow(fb) -> bool:
                        return abs(float(fb.get("speed_deg_s") or 0.0)
                                   ) < SLOW_DPS

                    now_temp: set[int] = set()
                    for j, fb in fb_map.items():
                        a = abs(float(fb.get("current_a") or 0.0))
                        if a > peak_a:
                            peak_a, peak_j = a, j
                        if a > HARD_CAP_A:
                            return _trip(
                                f"{joint_name(j)} at {a:.2f} A "
                                f"(hard cap {HARD_CAP_A:.1f} A)", label)
                        t_c = fb.get("temp_c")
                        if t_c is not None and int(t_c) >= TEMP_MAX_C:
                            # Debounced like the stall check below: one
                            # hot read is not trusted. Corrupted bytes on
                            # the shared bus fake 70-150 C spikes (08-11:
                            # "L4 hip at 150 °C" that read a steady 33 C
                            # seconds later killed a session; servo_watch
                            # documents the same 08-09 phantoms). Real
                            # heat survives two reads ~0.3-0.6 s apart.
                            if j in temp_prev:
                                return _trip(
                                    f"{joint_name(j)} at {int(t_c)} °C",
                                    label)
                            now_temp.add(j)
                    temp_prev = now_temp

                    now_stall = {
                        j for j, fb in fb_map.items()
                        if abs(float(fb.get("current_a") or 0.0)) > cur_lim
                        and _slow(fb) and errs[j] > 4.0}
                    hit = now_stall & stall_prev
                    if hit:
                        j = sorted(hit)[0]
                        return _trip(
                            f"stall-fight: {joint_name(j)} over "
                            f"{cur_lim:.1f} A while not moving "
                            f"({errs[j]:.0f}° from target){FORCE_HINT}",
                            label)
                    stall_prev = now_stall

                    now_load = {
                        j for j, fb in fb_map.items()
                        if float(fb.get("load_pct") or 0.0) > load_lim
                        and _slow(fb) and errs[j] > 4.0}
                    hit = now_load & load_prev
                    if hit:
                        j = sorted(hit)[0]
                        return _trip(
                            f"unexpected force: {joint_name(j)} load "
                            f"{float(fb_map[j]['load_pct']):.0f}% while "
                            f"not moving{FORCE_HINT}", label)
                    load_prev = now_load

                    el = last_fb - t0
                    for j, e in errs.items():
                        ref = progress_ref.get(j)
                        if ref is None or e < ref[1] - NO_PROGRESS_DEG:
                            progress_ref[j] = (el, e)
                            continue
                        if (e > NO_PROGRESS_MIN_ERR
                                and el - ref[0] > NO_PROGRESS_S):
                            return _trip(
                                f"{joint_name(j)} not turning: stuck "
                                f"{e:.0f}° from target for "
                                f"{el - ref[0]:.1f} s{FORCE_HINT}", label)

                    if el >= min_settle and worst_err <= SETTLE_DEG:
                        settled += 1
                        if settled >= 2:
                            break
                    else:
                        settled = 0
                    prog({"msg": (f"safe_zero {si + 1}/{n}: {label} · "
                                  f"err {worst_err:.1f}° · "
                                  f"peak {peak_a:.2f} A"),
                          "stage": si + 1, "of": n})
                time.sleep(0.08)

        return {"ok": True, "stages_done": n,
                "peak_a": round(peak_a, 2), "peak_joint": peak_j}
    finally:
        try:
            _set_torque_limit(bus, live, 1000)
        except Exception:
            pass
