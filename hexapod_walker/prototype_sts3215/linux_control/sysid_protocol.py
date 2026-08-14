"""Sysid protocol schema + deterministic materializer (stdlib only).

A *protocol* is a versioned JSON document describing a deterministic
command trajectory for system identification (HEXAPOD_SIM_TO_REAL_SYSID
plan, Phase 0/1). The SAME materializer runs on the laptop (sim replay,
fitting) and on the robot (sysid_runner), so hardware and simulation are
guaranteed to receive the identical per-tick command stream.

Schema (top level)::

    {
      "sysid_protocol": 1,
      "name": "steps_air_v1",
      "created": "2026-08-12T10:00:00",
      "description": "...",
      "hz": 25.0,                  # command/telemetry tick rate
      "write_speed": 400,          # Feetech profile speed (counts/s)
      "write_acc": 20,             # Feetech acc register units
      "soft_torque": 420,          # torque limit during the run
      "max_current_a": 0.9,        # per-joint trip (air default)
      "home_deg": [0.0, ...],      # optional 18-joint start pose: the
                                   # runner GLIDES there slowly before
                                   # the first segment (no hand-posing)
      "segments": [ ... ]
    }

Segment kinds (all command values in logical degrees):

- ``step``  — single joint; RELATIVE to the segment's home pose::

      {"kind": "step", "joint": 8, "amp_deg": 10.0,
       "pre_s": 0.6, "hold_s": 1.6, "rest_s": 0.8, "label": "..."}

- ``sine``  — single joint; RELATIVE, starts and ends at 0 phase::

      {"kind": "sine", "joint": 8, "amp_deg": 8.0, "freq_hz": 0.5,
       "cycles": 4, "pre_s": 0.6, "rest_s": 0.8, "label": "..."}

- ``traj``  — ABSOLUTE 18-joint stream (champion replay, Phase 8).
  The runner refuses unless the present pose matches ``q_deg[0]``
  within ``start_tol_deg`` (wrong zero frame protection)::

      {"kind": "traj", "t_s": [...], "q_deg": [[18 floats], ...],
       "start_tol_deg": 12.0, "label": "..."}

``materialize(protocol)`` expands the segments into a flat list of
ticks. Each tick is a dict::

    {"seg": i, "phase": "pre"|"move"|"rest",
     "mode": "rel"|"abs", "active": [joint, ...],
     "cmd": [18 floats]}      # rel: offset from segment home; abs: deg

The tick list is a pure function of the protocol document — no clock,
no randomness — so a protocol file hash identifies the experiment.
"""
from __future__ import annotations

import hashlib
import json
import math

PROTOCOL_VERSION = 1
N_JOINTS = 18
AXES = ("yaw", "hip", "knee")

# Mirror of feetech_bus.AXIS_LIMITS_DEG (kept literal so this module has
# zero imports; the runner re-checks against the live bus limits too).
AXIS_LIMITS_DEG = {
    "yaw": (-35.0, 35.0),
    "hip": (-80.0, 30.0),
    "knee": (-20.0, 150.0),
}

DEFAULT_HZ = 25.0
DEFAULT_WRITE_SPEED = 400      # counts/s — deployed RL runner profile
DEFAULT_WRITE_ACC = 20
DEFAULT_SOFT_TORQUE = 420
DEFAULT_MAX_CURRENT_A = 0.9    # air trip, same as motor_dynamics
MAX_REL_AMP_DEG = 40.0         # plan's largest step
MAX_TRAJ_TICKS = 4000          # ~160 s at 25 Hz


def axis_of(joint: int) -> str:
    return AXES[joint % 3]


def protocol_hash(protocol: dict) -> str:
    """Stable content hash identifying the experiment definition."""
    blob = json.dumps(protocol, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(blob.encode("utf-8")).hexdigest()[:12]


def validate(protocol: dict) -> list[str]:
    """Return a list of problems (empty = valid)."""
    errs: list[str] = []
    if protocol.get("sysid_protocol") != PROTOCOL_VERSION:
        errs.append(f"sysid_protocol != {PROTOCOL_VERSION}")
    hz = float(protocol.get("hz", DEFAULT_HZ))
    if not 5.0 <= hz <= 50.0:
        errs.append(f"hz {hz} outside 5..50")
    home = protocol.get("home_deg")
    if home is not None:
        if not isinstance(home, list) or len(home) != N_JOINTS:
            errs.append(f"home_deg must be {N_JOINTS} floats")
        else:
            for j, v in enumerate(home):
                lo, hi = AXIS_LIMITS_DEG[axis_of(j)]
                if not lo <= float(v) <= hi:
                    errs.append(f"home_deg[{j}]={v} outside [{lo}, {hi}]")
    segs = protocol.get("segments")
    if not isinstance(segs, list) or not segs:
        errs.append("no segments")
        return errs
    for i, s in enumerate(segs):
        kind = s.get("kind")
        if kind in ("step", "sine"):
            j = s.get("joint")
            if not isinstance(j, int) or not 0 <= j < N_JOINTS:
                errs.append(f"seg {i}: bad joint {j!r}")
                continue
            amp = float(s.get("amp_deg", 0.0))
            if not 0.0 < abs(amp) <= MAX_REL_AMP_DEG:
                errs.append(f"seg {i}: amp {amp} outside 0..±{MAX_REL_AMP_DEG}")
            if kind == "sine":
                f = float(s.get("freq_hz", 0.0))
                if not 0.05 <= f <= 3.0:
                    errs.append(f"seg {i}: freq {f} outside 0.05..3")
                if f > 0 and abs(amp) * 2.0 * math.pi * f > 720.0:
                    errs.append(f"seg {i}: peak sine velocity "
                                f"{abs(amp) * 2 * math.pi * f:.0f} deg/s > 720")
        elif kind == "traj":
            t = s.get("t_s")
            q = s.get("q_deg")
            if (not isinstance(t, list) or not isinstance(q, list)
                    or len(t) != len(q) or not t):
                errs.append(f"seg {i}: traj t_s/q_deg mismatch")
                continue
            if len(t) > MAX_TRAJ_TICKS:
                errs.append(f"seg {i}: traj {len(t)} ticks > {MAX_TRAJ_TICKS}")
            if any(len(row) != N_JOINTS for row in q):
                errs.append(f"seg {i}: traj rows must be {N_JOINTS} wide")
            else:
                for j in range(N_JOINTS):
                    lo, hi = AXIS_LIMITS_DEG[axis_of(j)]
                    vals = [row[j] for row in q]
                    if min(vals) < lo - 0.5 or max(vals) > hi + 0.5:
                        errs.append(f"seg {i}: traj joint {j} exceeds "
                                    f"axis limits [{lo}, {hi}]")
        else:
            errs.append(f"seg {i}: unknown kind {kind!r}")
    return errs


def _seg_label(s: dict, i: int) -> str:
    if s.get("label"):
        return str(s["label"])
    if s["kind"] == "step":
        return (f"j{s['joint']}_step{s['amp_deg']:+g}")
    if s["kind"] == "sine":
        return (f"j{s['joint']}_sine{s['amp_deg']:g}@{s['freq_hz']:g}Hz")
    return f"traj{i}"


def materialize(protocol: dict) -> dict:
    """Expand a protocol into the flat deterministic tick list.

    Returns ``{"hz", "ticks", "seg_labels"}``; see module docstring for
    the tick shape. Raises ValueError on an invalid protocol.
    """
    errs = validate(protocol)
    if errs:
        raise ValueError("invalid protocol: " + "; ".join(errs))
    hz = float(protocol.get("hz", DEFAULT_HZ))
    ticks: list[dict] = []
    labels: list[str] = []

    def n_ticks(seconds: float) -> int:
        return max(0, int(round(float(seconds) * hz)))

    for i, s in enumerate(protocol["segments"]):
        labels.append(_seg_label(s, i))
        kind = s["kind"]
        if kind == "traj":
            for row in s["q_deg"]:
                ticks.append({"seg": i, "phase": "move", "mode": "abs",
                              "active": list(range(N_JOINTS)),
                              "cmd": [float(v) for v in row]})
            continue
        j = int(s["joint"])
        amp = float(s["amp_deg"])
        pre = n_ticks(s.get("pre_s", 0.6))
        rest = n_ticks(s.get("rest_s", 0.8))
        zeros = [0.0] * N_JOINTS

        def tick(phase: str, rel: float) -> dict:
            cmd = list(zeros)
            cmd[j] = rel
            return {"seg": i, "phase": phase, "mode": "rel",
                    "active": [j], "cmd": cmd}

        for _ in range(pre):
            ticks.append(tick("pre", 0.0))
        if kind == "step":
            for _ in range(n_ticks(s.get("hold_s", 1.6))):
                ticks.append(tick("move", amp))
        else:  # sine
            f = float(s["freq_hz"])
            cyc = int(s.get("cycles", 3))
            total = n_ticks(cyc / f)
            for k in range(total):
                ticks.append(
                    tick("move", amp * math.sin(2.0 * math.pi * f * k / hz)))
        for _ in range(rest):
            ticks.append(tick("rest", 0.0))
    return {"hz": hz, "ticks": ticks, "seg_labels": labels}


def duration_s(protocol: dict) -> float:
    m = materialize(protocol)
    return len(m["ticks"]) / m["hz"]


def start_pose(protocol: dict) -> list[float] | None:
    """The 18-joint pose the runner glides to before the first segment.

    ``home_deg`` when given; else a leading traj segment's first row;
    else None (relative protocols may start from the present pose).
    """
    home = protocol.get("home_deg")
    if home is not None:
        return [float(v) for v in home]
    seg0 = protocol["segments"][0]
    if seg0.get("kind") == "traj":
        return [float(v) for v in seg0["q_deg"][0]]
    return None
