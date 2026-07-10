#!/usr/bin/env python3
"""Generate BuildViz MOTION data (joints + poses + keyframed animations) for
the prototype_v1 hexapod.

BuildViz already understands an additive kinematics block in ``scene.json``:

  * ``joints[]``  -- revolute/prismatic DOFs with a scene-frame ``axis`` +
                     ``origin`` (pivot) and the ``instances`` (distal link parts)
                     each one drives, composed along a ``parent`` chain.
  * ``poses[]``   -- named discrete joint-value configurations.
  * ``animations[]`` -- (added alongside this work) time-based keyframed joint
                     timelines the viewer plays with a play/pause + scrubber.

This module derives all three for the 6-leg / 3-DOF-per-leg hexapod so the
static CAD assembly can be seen WALKING.  It re-implements the compact tripod
gait + 2-link leg IK from ``mujoco_prototype.py`` (in millimetres, no mujoco /
numpy dependency) and the exact leg-placement geometry from ``inspect_build.py``
so the joint pivots land on the real CAD joint axes -- coxa yaw about +Z at the
hip mount, femur (hip) pitch and tibia (knee) pitch about the leg tangent -- and
the gait joint angles match the real robot's tucked 135 mm stance.

FK convention (must match BuildViz ``buildvizKinematics.ts``):
    L(joint, v) = T(origin) . R(axis, v - home) . T(-origin)   [revolute, deg]
    C(j)        = C(parent) . L(j)
    world(inst) = C(j) . base_transform(inst)     (base = the static scene pose)

The static scene (``home``) is the CAD's sprawled neutral
(hip = STANCE_FEMUR_DEG, knee = STANCE_TIBIA_DEG, yaw = 0); the walk clip drives
the joints to the tucked walking stance and cycles the tripod gait, so pressing
Play tucks the legs and steps forward.
"""

from __future__ import annotations

import math
from typing import Any


# ---------------------------------------------------------------------------
# Geometry + gait constants (all millimetres / radians), pulled from the same
# single sources of truth the CAD (hexapod_prototype) and the sim use.
# ---------------------------------------------------------------------------

def _load_constants() -> dict[str, float]:
    import hexapod_prototype as HP  # type: ignore

    return {
        "COXA": float(HP.COXA_LENGTH),
        "FEMUR": float(HP.FEMUR_LENGTH),
        "TIBIA": float(HP.TIBIA_LENGTH),
        "APOTHEM": float(HP.CHASSIS_FLAT_TO_FLAT) / 2.0,
        "YAW_OUTPUT_Z": float(HP.CHASSIS_YAW_OUTPUT_Z),
        "HIP_DROP": float(HP.COXA_HIP_DROP),
        "STANCE_FEMUR_DEG": float(HP.STANCE_FEMUR_DEG),
        "STANCE_TIBIA_DEG": float(HP.STANCE_TIBIA_DEG),
    }


# Firmware standing footprint (prototype_servo_test.ino STAND_TUCK_*): the real,
# tucked-in planted stance the gait plants on.  Matches mujoco_prototype.py.
STANCE_FOOT_RADIAL_MM = 135.0
STANCE_FOOT_Z_MM = -150.0

# Tripod gait parameters (visible forward walk in place).
GAIT_PERIOD_S = 0.65      # one full tripod cycle
GAIT_VX_MM_S = 150.0      # forward body velocity while walking
GAIT_LIFT_MM = 28.0       # swing-foot ground clearance
WALK_FRAMES = 48          # keyframes per gait cycle (seamless loop closes on frame N)
WALK_CLIP_SECONDS = 2.4   # clip playback length for one gait cycle


def _leg_angle(i: int) -> float:
    """Radial mounting angle of leg ``i`` (matches inspect_build / mujoco)."""
    return (i + 0.5) * math.pi / 3.0


def _leg_ik(x_radial: float, z: float, femur: float, tibia: float, coxa: float):
    """2-link planar IK in the leg's yaw frame.

    ``x_radial`` is the horizontal hip->foot distance (from the yaw axis) and
    ``z`` the foot height relative to the hip.  Returns ``(pitch, knee)`` where
    ``pitch`` is the femur angle (rad, +down) and ``knee`` the RELATIVE knee bend
    (rad).  ``None`` if the target is out of reach.  Mirror of
    ``mujoco_prototype._leg_ik`` (which returns the same two values).
    """
    u = x_radial - coxa
    w = -z
    L = math.hypot(u, w)
    if L > femur + tibia - 1e-6 or L < abs(femur - tibia) + 1e-6:
        return None
    cos_knee = (L * L - femur * femur - tibia * tibia) / (2 * femur * tibia)
    cos_knee = max(-1.0, min(1.0, cos_knee))
    knee = math.acos(cos_knee)
    pitch = math.atan2(w, u) - math.atan2(tibia * math.sin(knee), femur + tibia * math.cos(knee))
    return pitch, knee


def _stance_pitch_knee(c: dict[str, float]):
    """The tucked walking stance (rad) the gait plants on."""
    ik = _leg_ik(STANCE_FOOT_RADIAL_MM, STANCE_FOOT_Z_MM, c["FEMUR"], c["TIBIA"], c["COXA"])
    if ik is None:  # pragma: no cover - stance is always reachable by design
        return math.radians(-25.0), math.radians(60.0)
    return ik


# ---------------------------------------------------------------------------
# Tripod gait: per-leg (yaw, pitch, knee) as a function of a global phase.
# Ported from mujoco_prototype.TripodGait (ramp held at 1, vy = omega = 0).
# ---------------------------------------------------------------------------

def _gait_joint_values(phase: float, c: dict[str, float]) -> dict[str, tuple[float, float, float]]:
    """Return ``{ leg_index: (yaw_rad, pitch_rad, knee_rel_rad) }`` at ``phase``.

    ``phase`` sweeps 0..2*pi over one gait cycle.  The two tripods are offset by
    pi; each leg does a swing half (foot lifts + moves forward) then a stance
    half (foot planted, body drives it back).
    """
    p_stance, k_stance = _stance_pitch_knee(c)
    foot_neutral_x = STANCE_FOOT_RADIAL_MM         # hip->foot radial at neutral
    foot_neutral_z = STANCE_FOOT_Z_MM
    leg_radial = c["APOTHEM"]
    foot_radius = leg_radial + foot_neutral_x      # body-centre->foot radial
    t_eff = GAIT_PERIOD_S
    phase_offset = math.pi / 2.0

    out: dict[str, tuple[float, float, float]] = {}
    for i in range(6):
        a = _leg_angle(i)
        tripod = 0 if i % 2 == 0 else 1
        phi = (phase + phase_offset + tripod * math.pi) % (2 * math.pi)
        if phi < math.pi:            # swing: lift + advance
            s = phi / math.pi
            prog = -0.5 + s
            dz = GAIT_LIFT_MM * math.sin(math.pi * s)
        else:                         # stance: planted, drive back
            s = (phi - math.pi) / math.pi
            prog = 0.5 - s
            dz = 0.0
        dx = prog * GAIT_VX_MM_S * t_eff / 2.0

        ca, sa = math.cos(a), math.sin(a)
        fx_b = foot_radius * ca + dx
        fy_b = foot_radius * sa
        rx = fx_b - leg_radial * ca
        ry = fy_b - leg_radial * sa
        x_yaw = ca * rx + sa * ry
        y_yaw = -sa * rx + ca * ry
        yaw = math.atan2(y_yaw, x_yaw)
        r_planar = math.hypot(x_yaw, y_yaw)

        ik = _leg_ik(r_planar, foot_neutral_z + dz, c["FEMUR"], c["TIBIA"], c["COXA"])
        if ik is None:
            out[i] = (0.0, p_stance, k_stance)
        else:
            pitch, knee = ik
            out[i] = (yaw, pitch, knee)
    return out


# ---------------------------------------------------------------------------
# Joint pivots / axes in the SCENE frame (CAD frame + chassis lift in +Z),
# derived exactly like inspect_build._build_assembly_instances places the legs.
# ---------------------------------------------------------------------------

def _leg_joint_frames(i: int, c: dict[str, float], chassis_lift: float):
    a = _leg_angle(i)
    ca, sa = math.cos(a), math.sin(a)
    coxa, femur = c["COXA"], c["FEMUR"]
    hip_drop = c["HIP_DROP"]
    p0 = math.radians(c["STANCE_FEMUR_DEG"])   # CAD home femur pitch (rad)

    # yaw output (hip mount) in scene frame.
    yaw_pivot = [c["APOTHEM"] * ca, c["APOTHEM"] * sa, c["YAW_OUTPUT_Z"] + chassis_lift]

    # hip-pitch axis passes through (COXA, 0, hip_drop) in the leg-local frame.
    hip_pivot = [
        yaw_pivot[0] + coxa * ca,
        yaw_pivot[1] + coxa * sa,
        yaw_pivot[2] + hip_drop,
    ]

    # knee axis: leg-local (COXA + FEMUR*cos(p0), 0, hip_drop - FEMUR*sin(p0)).
    knee_local_x = coxa + femur * math.cos(p0)
    knee_local_z = hip_drop - femur * math.sin(p0)
    knee_pivot = [
        yaw_pivot[0] + knee_local_x * ca,
        yaw_pivot[1] + knee_local_x * sa,
        yaw_pivot[2] + knee_local_z,
    ]

    tangent = [-sa, ca, 0.0]   # R_z(a) . Y-hat -- the hip/knee swing axis
    return yaw_pivot, hip_pivot, knee_pivot, tangent


# ---------------------------------------------------------------------------
# Public entry point.
# ---------------------------------------------------------------------------

def build_motion(manifest_instances: list[dict[str, Any]], chassis_lift: float):
    """Build ``(joints, poses, animations)`` for the given scene instances.

    ``manifest_instances`` are the dicts export_buildviz already assembled (each
    with ``id`` / ``partType`` / ``leg`` / ``joint``).  Returns JSON-ready lists;
    if no legs are found (e.g. a chassis-only export) all three are empty.
    """
    c = _load_constants()

    # (partType, leg, joint) -> instance id, so joints reference real scene ids.
    lookup: dict[tuple[str, str | None, str | None], str] = {}
    legs: set[str] = set()
    for inst in manifest_instances:
        leg = inst.get("leg")
        key = (inst["partType"], leg, inst.get("joint"))
        lookup.setdefault(key, inst["id"])
        if leg:
            legs.add(leg)

    if not legs:
        return [], [], []

    def leg_index(leg: str) -> int:
        return int(leg[1:])   # "L3" -> 3

    home_hip = c["STANCE_FEMUR_DEG"]
    home_knee = c["STANCE_TIBIA_DEG"]

    joints: list[dict[str, Any]] = []
    joint_ids: list[str] = []   # keep a stable order for keyframes

    for leg in sorted(legs, key=leg_index):
        i = leg_index(leg)
        yaw_pivot, hip_pivot, knee_pivot, tangent = _leg_joint_frames(i, c, chassis_lift)

        coxa = lookup.get(("coxa_link", leg, None))
        femur = lookup.get(("femur_link", leg, None))
        tibia = lookup.get(("tibia_link", leg, None))
        yaw_horn = lookup.get(("servo_horn", leg, "yaw"))
        hip_body = lookup.get(("servo_body", leg, "hip"))
        hip_horn = lookup.get(("servo_horn", leg, "hip"))
        knee_body = lookup.get(("servo_body", leg, "knee"))
        knee_horn = lookup.get(("servo_horn", leg, "knee"))

        # Distal-link membership per DOF (the yaw servo BODY is chassis-fixed and
        # deliberately excluded; every other part rides the deepest joint above
        # it).  Missing parts (skipped STLs) are simply dropped.
        yaw_insts = [x for x in (yaw_horn, coxa, hip_body) if x]
        hip_insts = [x for x in (hip_horn, femur, knee_body) if x]
        knee_insts = [x for x in (knee_horn, tibia) if x]

        joints.append({
            "id": f"{leg}-yaw", "type": "revolute", "axis": [0.0, 0.0, 1.0],
            "origin": yaw_pivot, "instances": yaw_insts,
            "limits": {"min": -45.0, "max": 45.0}, "home": 0.0,
            "label": f"{leg} yaw",
        })
        joints.append({
            "id": f"{leg}-hip", "type": "revolute", "axis": tangent,
            "origin": hip_pivot, "parent": f"{leg}-yaw", "instances": hip_insts,
            "limits": {"min": -35.0, "max": 45.0}, "home": home_hip,
            "label": f"{leg} hip",
        })
        joints.append({
            "id": f"{leg}-knee", "type": "revolute", "axis": tangent,
            "origin": knee_pivot, "parent": f"{leg}-hip", "instances": knee_insts,
            "limits": {"min": 0.0, "max": 110.0}, "home": home_knee,
            "label": f"{leg} knee",
        })
        joint_ids += [f"{leg}-yaw", f"{leg}-hip", f"{leg}-knee"]

    ordered_legs = sorted(legs, key=leg_index)

    # ----- named poses (IK'd whole-body configurations) --------------------
    def pose_from_foot(x_radial: float, z: float) -> dict[str, float]:
        ik = _leg_ik(x_radial, z, c["FEMUR"], c["TIBIA"], c["COXA"])
        if ik is None:
            return {}
        pitch, knee = ik
        values: dict[str, float] = {}
        for leg in ordered_legs:
            values[f"{leg}-yaw"] = 0.0
            values[f"{leg}-hip"] = math.degrees(pitch)
            values[f"{leg}-knee"] = math.degrees(knee)
        return values

    stand = pose_from_foot(STANCE_FOOT_RADIAL_MM, STANCE_FOOT_Z_MM)  # 135 / -150
    crouch = pose_from_foot(120.0, -95.0)
    sit = pose_from_foot(105.0, -55.0)

    poses: list[dict[str, Any]] = [
        {"id": "sprawl", "name": "Sprawl (home)", "jointValues": {}},
        {"id": "stand", "name": "Stand", "jointValues": stand},
        {"id": "crouch", "name": "Crouch", "jointValues": crouch},
        {"id": "sit", "name": "Sit", "jointValues": sit},
    ]

    # ----- walk clip: tripod gait keyframes over one cycle -----------------
    walk_keyframes: list[dict[str, Any]] = []
    for k in range(WALK_FRAMES + 1):        # inclusive end closes the loop
        frac = k / WALK_FRAMES
        phase = 2 * math.pi * frac
        per_leg = _gait_joint_values(phase, c)
        values: dict[str, float] = {}
        for leg in ordered_legs:
            yaw, pitch, knee = per_leg[leg_index(leg)]
            values[f"{leg}-yaw"] = round(math.degrees(yaw), 3)
            values[f"{leg}-hip"] = round(math.degrees(pitch), 3)
            values[f"{leg}-knee"] = round(math.degrees(knee), 3)
        walk_keyframes.append({"t": round(frac * WALK_CLIP_SECONDS, 4), "jointValues": values})

    # ----- stand->sit clip: sequence the named poses -----------------------
    def full(values: dict[str, float]) -> dict[str, float]:
        filled: dict[str, float] = {}
        for leg in ordered_legs:
            filled[f"{leg}-yaw"] = values.get(f"{leg}-yaw", 0.0)
            filled[f"{leg}-hip"] = values.get(f"{leg}-hip", home_hip)
            filled[f"{leg}-knee"] = values.get(f"{leg}-knee", home_knee)
        return filled

    posture_steps = [full(stand), full(crouch), full(sit), full(crouch), full(stand)]
    posture_keyframes = [
        {"t": round(idx * 1.2, 4), "jointValues": step}
        for idx, step in enumerate(posture_steps)
    ]

    animations: list[dict[str, Any]] = [
        {
            "id": "walk",
            "name": "Walk (tripod gait)",
            "loop": True,
            "duration": WALK_CLIP_SECONDS,
            "keyframes": walk_keyframes,
        },
        {
            "id": "stand-sit",
            "name": "Stand \u2192 Sit",
            "loop": True,
            "duration": round((len(posture_steps) - 1) * 1.2, 4),
            "keyframes": posture_keyframes,
        },
    ]

    return joints, poses, animations
