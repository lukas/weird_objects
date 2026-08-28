"""Segment-relative grace windows for mode_seq safety terminations
(manual-drive-session-s1 dig-in, 2026-08-28).

The operator's manual-drive REPORT.md (finding #3) reproduced a
mid-session hold entry tripping `hold_min_load` in ~1.4s. Root cause:
`safety.hold_height_grace_s` / `hold_min_load_terminate_grace_s` /
`walk_height_grace_s` / `walk_idle_terminate_grace_s` were all gated
on the EPISODE-absolute clock (`self._step_i * self.dt`). A mode_seq
session's SECOND+ segment starts with `self._step_i` already large, so
any segment that begins after its own grace window would have elapsed
on a fresh single-mode episode gets ZERO grace instead of its own
window. Fixed by measuring elapsed time since `self._seg_entry_step`
(set at every `_seq_maybe_switch`, 0 for the whole episode when
mode_seq is off or for a fresh episode's first segment) instead of
since episode start. These tests lock:

1. a single-mode episode (mode_seq off) is bit-exact: grace measured
   from step 0 either way;
2. a mode_seq session's mid-episode hold segment gets its OWN full
   grace window (min-load termination cannot fire before
   grace elapses AFTER the switch, even though the episode clock is
   already past the grace threshold at switch time);
3. `_seg_entry_step` rides `mjx_host.SNAP_ATTRS` (pool-restore lesson).
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "linux_control"))
sys.path.insert(0, str(ROOT / "linux_control" / "urt2_setup"))

from rl_move.config import load_config  # noqa: E402
from rl_move.sim.mjx_host import SNAP_ATTRS  # noqa: E402
from rl_move.sim.walk_task import SimHexapodJointWalkEnv  # noqa: E402


def test_seg_entry_step_in_snap_attrs():
    assert "_seg_entry_step" in SNAP_ATTRS


def _make_env(seed=0, *, seq=False, episode_seconds=12.0,
              seg_s=(3.0, 3.0), extra_cfg=None):
    cfg = load_config()
    g = cfg.setdefault("goal", {})
    if seq:
        g["mode_seq"] = 1.0
        g["mode_seq_segment_s_min"] = seg_s[0]
        g["mode_seq_segment_s_max"] = seg_s[1]
    for k, v in (extra_cfg or {}).items():
        sect, name = k.split(".", 1)
        cfg.setdefault(sect, {})[name] = v
    return SimHexapodJointWalkEnv(cfg, seed=seed,
                                  episode_seconds=episode_seconds)


def test_seg_entry_step_zero_whole_episode_when_seq_off():
    env = _make_env(seq=False)
    env.reset()
    assert env._seg_entry_step == 0
    rng = np.random.default_rng(0)
    for _ in range(50):
        env.step(rng.uniform(-0.1, 0.1, env.action_space.shape[0]))
        assert env._seg_entry_step == 0


def test_midseq_hold_switch_resets_grace_clock():
    """Force a switch into 'hold' well past the episode-absolute grace
    window and confirm the min-load termination does NOT fire before
    the segment's OWN grace elapses (it must be measured from the
    switch tick, not from episode start)."""
    env = _make_env(
        seq=True, episode_seconds=20.0,
        extra_cfg={
            "safety.hold_min_load_terminate_s": 0.5,
            "safety.hold_min_load_terminate_grace_s": 2.0,
            "safety.hold_min_load_terminate_n": 100.0,  # every foot reads "low"
        },
    )
    env.reset()
    # Hand-build a plan: rise for 5s (episode clock already >> the 2s
    # hold grace by the time hold starts), then hold for the rest.
    dt = env.dt
    tk = lambda s: int(round(s / dt))  # noqa: E731
    env._seq_plan = [
        {"mode": "rise", "tick": 0, "blend": 0},
        {"mode": "hold", "tick": tk(5.0), "blend": 0},
    ]
    env._seq_idx = 0
    env._seq_seg_end = tk(5.0)
    rng = np.random.default_rng(1)
    reasons = []
    for _ in range(int(20.0 / dt) - 1):
        _, _, term, trunc, info = env.step(
            rng.uniform(-0.05, 0.05, env.action_space.shape[0]))
        reasons.append(info.get("termination_reason"))
        if term:
            break
    # The min-load EMA floor is set impossibly high (every foot reads
    # "low"), so once grace elapses the termination WILL fire — but it
    # must not fire before 2.0s have elapsed since the switch at 5.0s
    # (i.e. not before episode t=7.0s), which the old episode-absolute
    # clock (already at 5.0s >> 2.0s grace) would have violated
    # instantly at the very first hold tick.
    first_hold_tick = tk(5.0)
    fire_tick = None
    for i, r in enumerate(reasons):
        if r == "hold_min_load":
            fire_tick = i
            break
    assert fire_tick is not None, "expected hold_min_load to fire eventually"
    assert fire_tick >= first_hold_tick + tk(2.0) - 1, (
        f"hold_min_load fired at tick {fire_tick}, segment started at "
        f"{first_hold_tick} -- grace window was not honored "
        "(episode-absolute-clock regression)")
    env.close()
