"""Loader for `cpg_controller_*.json` artifacts (cpg track, 08-23).

The `cpg` track's search (`rl_move.sim.paper_cpg_search` +
`eval_cpg_gate.py`) exports its winning parameter vectors as
self-describing JSON (see `rl_move/sim/policies/cpg_controller_*.json`
for the schema: ``kind``, ``name``, ``params`` (gait/period/swing_frac/
lift_m/cmd_tau/workspace_margin), ``plant_stance_deg``, ``gate_summary``,
``provenance``). Nothing in the repo could read that schema back until
this module (cpg/STATUS.md Next item 4, grep-confirmed 08-23) — this is
the "consumable" half of the gate's own DONE-gate text, not a driving
change: it only parses/validates the artifact and turns it into
``SE2FootGait(**gait_kw)`` keyword arguments. Wiring it into a live
gait swap (``DriveController`` GAIT id 6 / ``CPGLOAD`` command) is a
separate, additive-only, default-inert change in `drive_controller.py`;
actually driving the physical robot with it is an [operator] hardware
decision, same standing rule as every other hardware-adjacent build
here (no HTTP/SSH/firmware touched by this module or its tests).
"""
from __future__ import annotations

import json
from pathlib import Path

HERE = Path(__file__).resolve().parent

# Search order: the deployable hardware-bench location first (mirrors
# bench_api.POLICIES_DIR's convention for RL policy JSON), then the
# sim-side/dev location where the cpg track's search actually writes
# its exports. Both are plain repo directories on the controller pod;
# only an operator's own deploy step decides what reaches the robot.
DEFAULT_DIRS = (
    HERE / "policies",
    HERE.parent / "rl_move" / "sim" / "policies",
)

KIND = "cpg_se2_controller"

# rl_move.sim.paper_cpg_search / build_motion_library param names ->
# linux_control.se2_foot_gait.SE2FootGait constructor kwarg names.
# Only ``lift_m`` differs (the artifact spells out units; the ctor
# param is plain ``lift`` in metres, same value).
_PARAM_TO_GAIT_KW = {
    "period": "period",
    "swing_frac": "swing_frac",
    "lift_m": "lift",
    "cmd_tau": "cmd_tau",
    "workspace_margin": "workspace_margin",
}
_REQUIRED_PARAMS = ("gait", "period", "swing_frac", "lift_m", "cmd_tau",
                    "workspace_margin")


def _search_dirs(dirs=None):
    if dirs is None:
        return DEFAULT_DIRS
    if isinstance(dirs, (str, Path)):
        return (Path(dirs),)
    return tuple(Path(d) for d in dirs)


def list_cpg_controllers(dirs=None) -> list[dict]:
    """List every ``cpg_controller_*.json`` found in ``dirs``.

    Later directories never shadow earlier ones by name (first match
    wins), mirroring ``bench_api.rl_policies``'s upload-shadows-repo
    rule. Each entry is a lightweight summary safe to send to the web
    UI directly; malformed files are reported, not raised.
    """
    out: list[dict] = []
    seen: set[str] = set()
    for d in _search_dirs(dirs):
        try:
            files = sorted(Path(d).glob("cpg_controller_*.json"))
        except OSError:
            continue
        for f in files:
            if f.name in seen:
                continue
            seen.add(f.name)
            entry: dict = {"file": f.name, "dir": str(d)}
            try:
                raw = json.loads(f.read_text())
            except Exception as e:  # noqa: BLE001 - report, don't raise
                entry["error"] = str(e)
                out.append(entry)
                continue
            if raw.get("kind") != KIND:
                entry["error"] = f"not a {KIND} artifact"
                out.append(entry)
                continue
            gate = raw.get("gate_summary") or {}
            dr0 = gate.get("dr0") or {}
            entry.update({
                "name": raw.get("name") or f.stem,
                "gait": (raw.get("params") or {}).get("gait"),
                "params": raw.get("params"),
                "gate_pass_dr0": dr0.get("pass"),
                "gate_slip_per_m": dr0.get("slip_per_m"),
                "gate_heading_progress_frac_mean":
                    dr0.get("heading_progress_frac_mean"),
                "provenance": raw.get("provenance"),
            })
            out.append(entry)
    out.sort(key=lambda r: r["file"])
    return out


def load_cpg_controller(name: str, dirs=None) -> dict:
    """Load + validate one artifact, returning ready-to-use gait kwargs.

    ``name`` may be a bare filename (``cpg_controller_foo.json``), a
    bare stem, or a full path. Returns
    ``{"name", "file", "gait", "gait_kw", "plant_stance_deg", "raw"}``
    where ``gait_kw`` is exactly the dict ``SE2FootGait(**gait_kw)``
    (or ``linux_control.sim_gait_compat.SE2FootGait`` for sim-relative
    knee replay) expects, alongside ``gait=gait_kw.pop("gait")``.
    Raises ``ValueError`` on any schema problem — this loader never
    silently falls back to defaults, so a bad artifact can't sneak a
    stale/no-op gait swap onto a live robot.
    """
    path = Path(name)
    if not path.is_file():
        candidates = []
        stem = path.stem if path.suffix else str(name)
        fname = stem if stem.startswith("cpg_controller_") \
            else f"cpg_controller_{stem}.json"
        if not fname.endswith(".json"):
            fname += ".json"
        for d in _search_dirs(dirs):
            candidates.append(Path(d) / fname)
        path = next((c for c in candidates if c.is_file()), None)
        if path is None:
            raise ValueError(
                f"no cpg controller artifact found for {name!r} "
                f"(tried {[str(c) for c in candidates]})")

    raw = json.loads(path.read_text())
    if raw.get("kind") != KIND:
        raise ValueError(
            f"{path}: kind={raw.get('kind')!r}, expected {KIND!r}")
    params = raw.get("params") or {}
    missing = [k for k in _REQUIRED_PARAMS if k not in params]
    if missing:
        raise ValueError(f"{path}: params missing keys {missing}")
    if params["gait"] not in ("tetrapod", "wave"):
        raise ValueError(f"{path}: unknown gait {params['gait']!r}")

    gait_kw = {gait_kw_name: float(params[param_name])
               for param_name, gait_kw_name in _PARAM_TO_GAIT_KW.items()
               if gait_kw_name != "gait"}
    return {
        "name": raw.get("name") or path.stem,
        "file": str(path),
        "gait": params["gait"],
        "gait_kw": gait_kw,
        "plant_stance_deg": raw.get("plant_stance_deg"),
        "raw": raw,
    }
