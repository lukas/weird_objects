"""Pinned-leg tip: detect it, and get out without cooking a motor.

THE scenario (bench_blast 08-11 evening, the overheat loop): a walk
safety-trips and the robot ends TIPPED with a knee folded under the
chassis, pressed against the ground at a big angle. Every absolute-pose
recovery that follows (safe-zero glide, learned stand, scripted stand)
drives 18 joints at full torque limit against that pin — each attempt
stalls somewhere, limps, gets retried, and the heat stacks (L2 hip
71 °C and L4 hip 68 °C mid-glide with shutoff at 65). The per-joint
stall trips all fired correctly; what was missing is anything that
RECOGNIZES the trapped state and refuses to fight it.

Two halves, same split as safe_zero.py:

DETECTOR (read-only — safe to call before ANY motion):

``check_pinned_tip(bus)`` — one IMU read + one encoder sweep; only
when tipped does it wait ``SETTLE_S`` and read again (a transient
gait rock must not classify).  Verdict fields:

  tipped  body tilt (from the gravity vector) >= ``TIP_DEG`` on both
          reads.
  pinned  tipped AND at least one knee folded >= ``PIN_KNEE_DEG``.
          A folded knee under a tipped body is the prop/trap
          signature: after a fall from the plant stance all knees
          read ~79°, so real falls classify pinned, while a
          legs-straight tip (robot on a slope, or placed on its
          side) stays tipped-but-NOT-pinned and keeps the existing
          operator-judgment path.

``classify_pinned_tip(q_deg, roll_deg, pitch_deg)`` is the pure
decision (no hardware) so it can be unit-tested and reused by
read-only preflights that already hold a pose + tilt.

ESCAPE — ``run_untrap_tuck(bus)``: the one move that is safe against
an unknown pin.  Limp → gravity settle → torque limit ``TUCK_TORQUE``
(20 %) → one slow glide of hips+knees to a fold pose (yaws held).
The determinism comes from the torque limit, not from clever
monitoring: at 20 % a stalled STS3215 draws a few hundred mA, which
it can hold indefinitely — the escape can FAIL but it cannot make
heat.  Folding pulls every tibia lever in toward the body, so
whatever props the chassis up collapses and the body rolls flat;
the pinned tibia slides out instead of jacking against the ground.
Success = tilt back under ``LEVEL_DEG``.  Joints that never tracked
the fold are reported by name — that is the reliable identification
of WHICH joint was trapped (measured during the safe move itself).
Failure limps and stops: never retried, torque never escalated — a
robot this routine cannot free needs the operator's hands.

Surprise-force rule (operator, 08-11): a mechanical jam or a wrong
logical zero is ALWAYS possible, so the escape refuses up front when
any encoder sits outside its axis range by > ``UNTRAP_LIMIT_SLOP_DEG``
(zero frame suspect → set_zero first), and during the fold any joint
holding more current than the 20 % limit should allow trips an
immediate limp with an error naming the joint and the likely causes.
Quiet, torque-bounded stalls on the pinned leg stay expected and
tolerated; force that CONTRADICTS the torque limit is never fought.
"""
from __future__ import annotations

import math
import time

from feetech_bus import AXIS_LIMITS_DEG, N_JOINTS, joint_to_servo_id
from safe_zero import SLOW_DPS, foot_z_mm, joint_name

# Zero-frame sanity margin for the pre-fold check. WIDER than
# safe_zero's 20° slop on purpose (live lesson, 08-11 21:05): in a real
# trapped state the body's weight legitimately shoves a pinned hip 25°+
# past its soft limit (measured L3 hip +58° vs the +30° limit), which
# is exactly when untrap is needed — a 20° slop refused it as "wrong
# zero". 40° still catches genuinely wild zero frames (90-180° off),
# and a moderately wrong zero is survivable HERE because the fold runs
# at 20 % torque with the surprise-force trip armed; full-torque
# safe_zero keeps its stricter 20°.
UNTRAP_LIMIT_SLOP_DEG = 40.0

TIP_DEG = 12.0          # body tilt that counts as tipped. Calibrated
                        # against the REAL post-fall rest states: the
                        # 08-11 bench falls settled at 8.7-16.7° tail
                        # lean, and the first live pinned test (08-11
                        # 21:05) propped at 13° — the original 20°
                        # missed every one of them. 12° matches the
                        # rl preflight's "not startable" line; IMU
                        # mounting bias measures ~3-4°.
LEVEL_DEG = 8.0         # untrap success: tilt back under this
                        # (must sit below TIP_DEG with margin)
PIN_KNEE_DEG = 45.0     # knee flexion that can prop/trap under the body
SETTLE_S = 1.2          # between the two tipped-confirmation reads

# Escape (fold) targets: inside the sim-validated scripted-tuck ball
# (hip −78 / knee +148) with margin to the axis limits, so a fully
# tracking fold is never a limit crash.
FOLD_HIP_DEG = -50.0
FOLD_KNEE_DEG = 140.0
TUCK_TORQUE = 200       # /1000 — the heat bound; NEVER raise to "help"
REST_S = 2.0            # limp gravity-settle before the fold
TUCK_S = 8.0            # commanded glide duration
TUCK_TIMEOUT_S = 16.0   # then we stop waiting and judge by tilt
TRACK_TOL_DEG = 15.0    # a joint further than this from the fold at the
                        # end is reported as the trapped one
SETTLE_TOL_DEG = 4.0    # everyone this close = fold done early
# Surprise force (operator rule 08-11): a mechanical jam or a WRONG
# LOGICAL ZERO is always possible, so unexpected force = stop, limp,
# say which joint. At torque limit 200/1000 an honestly pinned joint
# stalls around ~1.4 A (20 % of the ~7 A full-stall); sustained
# current past this on a non-moving joint means the fight is NOT
# torque-bounded as designed — most likely the limit write silently
# failed (_set_torque_limit swallows bus errors), or the zero frame
# is wrong and the servo is grinding a hard stop. Two consecutive
# sweeps (house debounce — one corrupted read must not trip).
SURPRISE_CURRENT_A = 2.0
HARD_CAP_A = 3.5        # any single reading past this = same trip, no wait
TEMP_MAX_C = 63


def tilt_from_imu(imu) -> tuple[float, float] | None:
    """(roll_deg, pitch_deg) from an accel reading, or None when the
    reading is missing/implausible (same |g| sanity as rl preflight —
    a dead MPU reads zeros and atan2(0,0)=0 would false-pass)."""
    if not isinstance(imu, dict) or "ax_g" not in imu:
        return None
    ax = float(imu.get("ax_g") or 0.0)
    ay = float(imu.get("ay_g") or 0.0)
    az = float(imu.get("az_g") or 0.0)
    mag = math.sqrt(ax * ax + ay * ay + az * az)
    if not 0.5 <= mag <= 1.5:
        return None
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
    return roll, pitch


def classify_pinned_tip(q_deg, roll_deg: float, pitch_deg: float) -> dict:
    """Pure verdict from a pose + tilt (no hardware, unit-testable).

    ``q_deg``: 18 present joint degrees (None entries allowed for dead
    encoders — a missing knee makes the pose unclassifiable and the
    verdict stays un-pinned so callers fall back to their existing
    conservative gates).
    """
    tilt = max(abs(float(roll_deg)), abs(float(pitch_deg)))
    out: dict = {
        "tipped": tilt >= TIP_DEG,
        "pinned": False,
        "tilt_deg": round(tilt, 1),
        "roll_deg": round(float(roll_deg), 1),
        "pitch_deg": round(float(pitch_deg), 1),
        "candidates": [],
    }
    if not out["tipped"]:
        out["why"] = f"level enough (tilt {tilt:.0f}° < {TIP_DEG:.0f}°)"
        return out
    knees_known = True
    for leg in range(6):
        hip = q_deg[leg * 3 + 1]
        knee = q_deg[leg * 3 + 2]
        if knee is None:
            knees_known = False
            continue
        if float(knee) >= PIN_KNEE_DEG:
            out["candidates"].append({
                "leg": leg,
                "joint": leg * 3 + 2,
                "name": joint_name(leg * 3 + 2),
                "knee_deg": round(float(knee), 1),
                # where this foot would sit if the body were level —
                # far above ground + a tipped body = resting ON the leg
                "foot_z_mm": (round(foot_z_mm(float(hip), float(knee)), 1)
                              if hip is not None else None),
            })
    if not knees_known and not out["candidates"]:
        out["why"] = "tipped but knee encoders missing — cannot classify"
        return out
    if out["candidates"]:
        out["pinned"] = True
        names = ", ".join(c["name"] for c in out["candidates"])
        out["why"] = (f"tipped {tilt:.0f}° with folded knee(s) under it "
                      f"({names}) — pinned-leg tip")
    else:
        out["why"] = (f"tipped {tilt:.0f}° but all knees near straight — "
                      "slope or hand-placed; not the pinned-leg trap")
    return out


def _read_q(bus) -> list:
    """18 present degrees (None where a joint gave nothing)."""
    q: list = [None] * N_JOINTS
    try:
        fb = bus.read_all_feedback() or {}
    except Exception:
        fb = {}
    for j, f in fb.items():
        if 0 <= j < N_JOINTS and f.get("deg") is not None:
            q[j] = float(f["deg"])
    if any(v is None for v in q) and hasattr(bus, "read_all_positions"):
        try:
            for j, v in (bus.read_all_positions() or {}).items():
                if 0 <= j < N_JOINTS and q[j] is None:
                    q[j] = float(v)
        except Exception:
            pass
    return q


def check_pinned_tip(bus, *, settle_s: float = SETTLE_S) -> dict:
    """READ-ONLY detector — safe to call before any motion.

    Fast path: one IMU read; a level robot returns immediately.
    Tipped: wait ``settle_s``, read IMU + encoders again, and classify
    on the second (settled) reading — both reads must agree on tipped.
    Never commands, never limps.  IMU trouble reports
    ``pinned=False`` + ``error`` so callers keep their existing gates.
    """
    if bus is None or not hasattr(bus, "read_imu"):
        return {"pinned": False, "tipped": False, "error": "no IMU on bus"}
    try:
        t1 = tilt_from_imu(bus.read_imu())
    except Exception as e:
        return {"pinned": False, "tipped": False, "error": f"IMU: {e}"}
    if t1 is None:
        return {"pinned": False, "tipped": False,
                "error": "IMU not answering / implausible"}
    if max(abs(t1[0]), abs(t1[1])) < TIP_DEG:
        return classify_pinned_tip([None] * N_JOINTS, t1[0], t1[1])

    q_first = _read_q(bus)
    time.sleep(settle_s)
    try:
        t2 = tilt_from_imu(bus.read_imu())
    except Exception:
        t2 = None
    if t2 is None or max(abs(t2[0]), abs(t2[1])) < TIP_DEG:
        # transient rock (or the IMU blinked): do not classify pinned
        v = classify_pinned_tip([None] * N_JOINTS,
                                *(t2 if t2 is not None else t1))
        v["tipped"] = False
        v["pinned"] = False
        v["why"] = "tilt did not persist across the settle read"
        return v
    q = _read_q(bus)
    v = classify_pinned_tip(q, t2[0], t2[1])
    v["settling_max_deg"] = round(max(
        (abs(a - b) for a, b in zip(q_first, q)
         if a is not None and b is not None), default=0.0), 1)
    return v


def run_untrap_tuck(bus, *, abort_check=None, on_progress=None) -> dict:
    """Deterministic escape from a pinned-leg tip (see module doc).

    Limp → settle → 20 % torque limit → one slow fold of hips+knees
    (yaws held) → judge by tilt.  On success the robot is left level,
    folded, holding at the LOW limit (the follow-up safe_zero sets its
    own 700); on failure or abort it is left LIMP — holding a fight is
    exactly the failure mode this module exists to prevent.
    """
    from inplace_demos import (_enable_torque, _glide_speed_acc, _limp_all,
                               _live_robot_ids, _read_pose,
                               _set_torque_limit, _write_pose)
    check = abort_check or (lambda: False)
    prog = on_progress or (lambda d: None)

    live = _live_robot_ids(bus)
    missing = [joint_to_servo_id(j) for j in range(N_JOINTS)
               if joint_to_servo_id(j) not in live]
    if missing:
        return {"ok": False,
                "error": (f"servo IDs {missing} not answering — untrap "
                          "needs all 18 joints for its monitoring")}

    # Zero-frame sanity BEFORE any motion (same rule as plan_safe_zero
    # but with the wider UNTRAP_LIMIT_SLOP_DEG — see its comment): an
    # encoder outside its axis range by more than the slop means the
    # logical zero is untrustworthy — the fold targets would be
    # meaningless absolute angles. Refuse without touching anything.
    for j, v in enumerate(_read_pose(bus, live)):
        lo, hi = AXIS_LIMITS_DEG[j % 3]
        if (v < lo - UNTRAP_LIMIT_SLOP_DEG
                or v > hi + UNTRAP_LIMIT_SLOP_DEG):
            return {"ok": False, "joint": j, "code": "suspect_zero",
                    "error": (f"{joint_name(j)} reads {v:+.1f}° — outside "
                              f"its {lo:.0f}..{hi:.0f}° range by more than "
                              f"{UNTRAP_LIMIT_SLOP_DEG:.0f}°. The logical "
                              "zero frame looks wrong; hand-set the known "
                              "pose and POST /api/set_zero before any "
                              "untrap or absolute move.")}

    prog({"msg": "untrap: limp + gravity settle"})
    _limp_all(bus, live)
    time.sleep(REST_S)

    present = _read_pose(bus, live)
    yaw_lo, yaw_hi = AXIS_LIMITS_DEG[0]
    goal = list(present)
    for leg in range(6):
        goal[leg * 3] = min(yaw_hi, max(yaw_lo, present[leg * 3]))
        goal[leg * 3 + 1] = FOLD_HIP_DEG
        goal[leg * 3 + 2] = FOLD_KNEE_DEG

    result: dict = {"torque_limit": TUCK_TORQUE, "peak_a": 0.0}

    def _fail(reason: str) -> dict:
        try:
            _limp_all(bus, live)
        except Exception:
            pass
        try:
            _set_torque_limit(bus, live, 1000)   # torque is OFF; restore
        except Exception:                        # the expected default
            pass
        result.update(ok=False, limp=True, error=reason)
        return result

    _set_torque_limit(bus, live, TUCK_TORQUE)
    _enable_torque(bus, live)
    speed, acc = _glide_speed_acc(present, goal, live, TUCK_S)
    prog({"msg": f"untrap: folding at {TUCK_TORQUE / 10:.0f}% torque"})
    _write_pose(bus, goal, live, speed=speed, acc=acc)

    t0 = time.monotonic()
    errs: dict[int, float] = {}
    temp_prev: set[int] = set()
    surprise_prev: set[int] = set()
    while True:
        if check():
            return _fail("operator abort")
        el = time.monotonic() - t0
        if el > TUCK_TIMEOUT_S:
            break                       # pinned joints never arrive — fine
        fb = {}
        try:
            fb = bus.read_all_feedback() or {}
        except Exception:
            pass
        if fb:
            errs = {j: abs(goal[j] - float(f["deg"]))
                    for j, f in fb.items() if f.get("deg") is not None}
            now_temp: set[int] = set()
            now_surprise: set[int] = set()
            for j, f in fb.items():
                a = abs(float(f.get("current_a") or 0.0))
                if a > result["peak_a"]:
                    result["peak_a"] = round(a, 2)
                if a > HARD_CAP_A:
                    return _fail(
                        f"surprise force: {joint_name(j)} at {a:.2f} A "
                        f"(hard cap {HARD_CAP_A:.1f} A) despite the "
                        f"{TUCK_TORQUE / 10:.0f}% torque limit — "
                        "mechanical jam, wrong zero (re-do set_zero), "
                        "or the limit write failed; not pushing through")
                if (a > SURPRISE_CURRENT_A
                        and abs(float(f.get("speed_deg_s") or 0.0))
                        < SLOW_DPS):
                    now_surprise.add(j)
                t_c = f.get("temp_c")
                if t_c is not None and int(t_c) >= TEMP_MAX_C:
                    # Two consecutive hot reads required — single-read
                    # trips fire on corrupted-bus phantom temps (08-11:
                    # a fake "150 °C" that read 33 C seconds later).
                    if j in temp_prev:
                        return _fail(f"{joint_name(j)} at {int(t_c)} °C")
                    now_temp.add(j)
            temp_prev = now_temp
            hit = now_surprise & surprise_prev
            if hit:
                j = sorted(hit)[0]
                a = abs(float(fb[j].get("current_a") or 0.0))
                return _fail(
                    f"surprise force: {joint_name(j)} holding {a:.2f} A "
                    "while not moving — more than the "
                    f"{TUCK_TORQUE / 10:.0f}% torque limit should allow. "
                    "Possible mechanical jam or wrong zero (re-do "
                    "set_zero), or the limit write failed; not pushing "
                    "through")
            surprise_prev = now_surprise
            worst = max(errs.values(), default=0.0)
            if el >= TUCK_S * 0.6 and worst <= SETTLE_TOL_DEG:
                break                   # clean fold, everyone arrived
            prog({"msg": (f"untrap: folding · worst err {worst:.0f}° · "
                          f"peak {result['peak_a']:.2f} A")})
        time.sleep(0.3)

    trapped = sorted(j for j, e in errs.items() if e > TRACK_TOL_DEG)
    result["trapped_joints"] = trapped
    result["trapped_names"] = [joint_name(j) for j in trapped]

    tilt = None
    for _ in range(3):
        try:
            t = tilt_from_imu(bus.read_imu())
        except Exception:
            t = None
        if t is not None:
            tilt = max(abs(t[0]), abs(t[1]))
            break
        time.sleep(0.3)
    if tilt is None:
        return _fail("IMU unreadable after fold — cannot verify level")
    result["tilt_deg"] = round(tilt, 1)

    if tilt > LEVEL_DEG:
        who = (", ".join(result["trapped_names"])
               or "no joint clearly blocked")
        return _fail(f"still tipped {tilt:.0f}° after the low-torque "
                     f"fold ({who}) — reposition by hand")
    result.update(ok=True, seconds=round(time.monotonic() - t0, 1))
    prog({"msg": (f"untrap: level ({tilt:.0f}°), folded, holding at "
                  f"{TUCK_TORQUE / 10:.0f}% torque")})
    return result
