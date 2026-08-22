"""RSI-spawn height-schedule target (08-22 root-cause fix).

cw-stand-footlow2-plant150-2c-heightfix's gate showed RSI-start rise
episodes pinned at 22-29mm height error, UNMOVED by 10M extra steps of
training (bridge/crouch/flat all clean at 1.3-3.9mm, unchanged). Root
cause: goal.rise_rsi_frac's height-schedule rewrite (sim_env.py, the
post-settle re-anchor block) anchored the episode's ABSOLUTE height
TARGET to the reference npz's own recorded final height
(``ref["h"][-1]``), which is stale relative to the current sim
geometry (rise_ref_belly2plant.npz was extracted before the tibia-150
change: stored h_rel_end_m=110.96mm vs the SAME q_rad trajectory's
actually-measured settle height of 131.94mm on the current sim --
CURRENT_TRUTHS). Every RSI episode was therefore trained toward a
target ~21mm below the real goal.rise_height_mm window -- a genuine
reward<->eval misalignment, not undertraining.

Fix: anchor the target height to the episode's own already-sampled,
CURRENT-cfg schedule (``self._goal_traj.height[-1]``, drawn from
goal.rise_height_mm before the RSI block runs) and use the reference
array only for the FRACTIONAL progress at the spawn point.

Contract under test:
  - an RSI episode's rewritten height schedule must top out at (or
    below, if the spawn point isn't the very start of the ramp) the
    CURRENT cfg target, never pinned at a stale reference height that
    is far below the target;
  - the schedule never OVERSHOOTS the current target either;
  - default (rise_rsi_frac=0) path is completely untouched (existing
    test_rise_start_bank.py coverage) -- not re-asserted here.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "linux_control"))
sys.path.insert(0, str(ROOT / "linux_control" / "urt2_setup"))

pytest.importorskip("mujoco")

from rl_move.config import load_config                      # noqa: E402
from rl_move.sim.goal_task import SimHexapodGoalEnv         # noqa: E402
from rl_move.sim.sim_env import N_JOINTS                    # noqa: E402


def _mk_stale_ref(tmp_path: Path, h_end_stale: float,
                  n: int = 60, i0: int = 10) -> Path:
    """A synthetic belly->plant reference whose OWN recorded height
    column tops out far below a plausible current rise_height_mm
    target -- models rise_ref_belly2plant.npz's pre-tibia-150 staleness
    without depending on the real (large, geometry-specific) file."""
    q = np.zeros((n, N_JOINTS))
    curl = np.linspace(0.0, 1.0, n - i0)
    q[i0:, 2::3] = 0.30 + 0.5 * curl[:, None]
    h = np.zeros(n)
    h[i0:] = np.linspace(0.0, h_end_stale, n - i0)
    p = tmp_path / "stale_rise_ref.npz"
    np.savez(p, q_rad=q, dt=0.04, ramp_i0=i0, h_rel_m=h,
             h_rel_end_m=h_end_stale)
    return p


def _rsi_env(seed: int, ref_path: Path, target_mm: float,
            max_height_mm: float = 150.0) -> SimHexapodGoalEnv:
    cfg = load_config()
    cfg.setdefault("actions", {})["max_height_mm"] = max_height_mm
    cfg.setdefault("goal", {})
    cfg["goal"]["rise_height_mm"] = [target_mm, target_mm]
    cfg["goal"]["rise_rsi_frac"] = 1.0
    cfg["goal"]["rise_hold_s"] = 1.0
    cfg["goal"]["rise_hold_min_s"] = 1.0
    cfg.setdefault("reward", {})["rise_ref_path"] = str(ref_path)
    env = SimHexapodGoalEnv(cfg=cfg, seed=seed)
    g = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad", "walk"):
        if hasattr(g, f"p_{m}"):
            setattr(g, f"p_{m}", 1.0 if m == "rise" else 0.0)
    return env


def test_rsi_schedule_targets_current_height_not_stale_ref(tmp_path):
    """The bug: h_left was capped by the reference's own stale h[-1]
    (0.040m here) even though the live cfg target is 0.130m. The fix
    must let at least one early-spawn episode's schedule climb well
    past the stale cap, toward the real target."""
    target_mm = 130.0
    h_end_stale = 0.040  # far below 0.130m -- models the ~21mm-ish
                          # real-world gap, exaggerated for a clear
                          # signal
    ref_path = _mk_stale_ref(tmp_path, h_end_stale)
    env = _rsi_env(seed=5, ref_path=ref_path, target_mm=target_mm)
    ends_m = []
    rsi_seen = 0
    for _ in range(40):
        env.reset()
        if getattr(env, "_rsi_pending", False):
            rsi_seen += 1
            ends_m.append(float(np.asarray(env._goal_traj.height)[-1]))
    env.close()
    assert rsi_seen >= 5, "RSI never fired at rise_rsi_frac=1.0"
    assert ends_m, "no RSI schedule heights collected"
    # Never overshoots the live target.
    assert max(ends_m) <= target_mm * 0.001 + 1e-6, (
        f"RSI schedule overshot the current target: "
        f"{max(ends_m)*1000:.1f}mm > {target_mm}mm")
    # The bug's signature: EVERY episode pinned at/under the stale
    # reference cap. The fix must break that ceiling for at least the
    # early-spawn episodes (frac_done near 0 -> h_left near the full
    # live target).
    assert max(ends_m) > h_end_stale + 0.010, (
        f"RSI schedule still pinned near the stale reference height "
        f"({max(ends_m)*1000:.1f}mm vs stale cap "
        f"{h_end_stale*1000:.1f}mm) -- the stale-reference-height bug "
        f"is back"
    )


def test_rsi_schedule_scales_with_live_target(tmp_path):
    """Two runs, same seed/ref (so the same spawn indices are drawn),
    different rise_height_mm targets: the schedule ends must scale
    with the LIVE target, not sit at a fixed value set by the
    reference file."""
    h_end_stale = 0.040
    ref_path = _mk_stale_ref(tmp_path, h_end_stale)
    ends_low, ends_high = [], []
    for target_mm, bucket in ((80.0, ends_low), (150.0, ends_high)):
        env = _rsi_env(seed=9, ref_path=ref_path, target_mm=target_mm,
                       max_height_mm=160.0)
        for _ in range(20):
            env.reset()
            if getattr(env, "_rsi_pending", False):
                bucket.append(float(np.asarray(env._goal_traj.height)[-1]))
        env.close()
    assert ends_low and ends_high, "RSI never fired in one of the arms"
    assert max(ends_high) > max(ends_low), (
        "raising the live rise_height_mm target did not raise the RSI "
        "schedule's ceiling -- target is still pinned to the reference "
        "file instead of the live cfg"
    )
