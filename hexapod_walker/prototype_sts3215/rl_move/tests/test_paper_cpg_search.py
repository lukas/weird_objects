"""Fast checks for the Berkeley-style CPG gait search runner."""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from rl_move.sim.paper_cpg_search import score_rollouts  # noqa: E402
from sim_gait_compat import SE2FootGait  # noqa: E402


def test_se2_compat_outputs_sim_relative_knees():
    g = SE2FootGait(gait="tetrapod", vx=0.0, vy=0.0, omega=0.0)
    g.sync_plant_stance(20.0, 80.0)
    q = g.desired_deg(0.0)
    for leg in range(6):
        hip = q[3 * leg + 1]
        knee = q[3 * leg + 2]
        assert abs(hip - 20.0) < 1e-6
        assert abs(knee - 80.0) < 1e-6


def test_score_prefers_progress_low_slip_no_fall():
    base = {
        "progress_frac": 0.95,
        "cross_frac": 0.05,
        "slip_per_m": 0.35,
        "roll_peak_deg": 3.0,
        "pitch_peak_deg": 3.0,
        "current_p95_a": 1.0,
        "terminated": "",
        "command_scale": 1.0,
    }
    slippery = dict(base, slip_per_m=3.0)
    wrong_way = dict(base, progress_frac=-0.25, cross_frac=1.0)
    fallen = dict(base, terminated="roll")

    good_s = score_rollouts([base])["score"]
    slip_s = score_rollouts([slippery])["score"]
    wrong_s = score_rollouts([wrong_way])["score"]
    fall_s = score_rollouts([fallen])["score"]

    assert good_s > slip_s
    assert good_s > wrong_s
    assert good_s > fall_s
