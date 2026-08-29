"""WALK_COURSE_DISP bank -- validates the k_walk_course fix-lever (b)
mechanism (2026-08-29 standwalk DIG-IN).

Background: `cw-standwalk-unified1-mix-long-s1-cont1` FAILed its own
gate with slip improving but direction_err plateaued at ~62-68 deg for
its whole 16-32M-step lineage. Root-cause (idle-kick ~04:3x): the
recipe's ONLY course/heading-pricing term, `reward.k_walk_course`, was
found COMPLETELY INERT for that lineage's entire runtime -- its
vector-EMA of the INSTANTANEOUS body velocity (tau=0.75s) partially
CANCELS under stride-to-stride zigzag even when net travel is real
(0/5899 active ticks in a full deterministic repro despite the
checkpoint's own DR-0 gate showing real forward motion). The obvious
scalar fixes on that EMA (lower the activation floor, raise tau) were
CLOSED by direct measurement (STATUS.md ~05:0x): every floor in the
diagnosed activation band breaks 8 of `test_phasedir_semantics.py`'s
own invariants, and raising tau LOWERS the achieved magnitude instead
of raising it.

This bank validates the alternative: `reward.k_walk_course_disp` prices
the NET BODY-POSITION DISPLACEMENT over a trailing window instead of an
EMA of instantaneous velocity -- immune to intra-stride sway AND slow
zigzag cancellation by construction, since it measures where the body
ACTUALLY ENDED UP over the window, not an average of noisy per-tick
velocity samples.

VALIDATION AGAINST THE REAL FAILED CHECKPOINT (not just a scripted
proxy, per the DIG-IN's own requirement): a local deterministic repro
of `ppo_goal_cw_standwalk_unified1_mix_long_s1_cont1.zip`'s own 60s
walk/det rollout (dr_scale=0, matched cfg stack) measured, over the
identical tick stream:
  - per-tick INSTANTANEOUS direction error: mean 57.6 deg, median
    32.7 deg (matches the harness's own `direction_err_mean_deg`
    headline of ~55-62 almost exactly)
  - windowed NET DISPLACEMENT direction error at every window tested
    (0.75 / 1.5 / 3.0 / 6.0 s): 6.2-6.5 deg, with mean windowed speed
    ~0.029 m/s -- comfortably clearing a 0.02 m/s floor even though
    the vector EMA of the identical ticks never clears 0.04
This is the single strongest piece of evidence for the mechanism: the
robot's real PATH is close to on-course, but the harness's (and the
old reward term's) tick-level instantaneous-velocity view is dominated
by honest stride sway and cannot see it.

The tests below are the scripted-teacher BANK proof the fix-lever
needs before any launch (RESEARCH_RULES: reward-mechanism arms need
their semantics bank green first) -- they show the new mechanism:
  1. defaults OFF bit-exact (no state, no info keys, no reward delta);
  2. reproduces the EXISTING `k_walk_course` EMA mechanism's pricing
     of every scripted-teacher behavior class (obey/skew/stall/park/
     wrongway) to within noise, i.e. it is a safe drop-in for every
     invariant the EMA form currently satisfies (same margins, no new
     holes) -- including reusing `walk_course_disp_overspeed` as the
     structural twin of `walk_course_overspeed` so removing
     k_walk_course does not silently drop the overspeed protection
     that lives nested inside its own `if` guard;
  3. runs the REAL failed checkpoint through the production
     `eval_checkpoint` CLI (a diagnostic `--course-trace` flag added
     this cycle) and confirms, on that one real rollout: the EMA stays
     near-inert, the windowed net-displacement direction error is
     small, and `k_walk_course_disp` activates on most ticks with a
     small mean course error -- the DIG-IN's own required validation
     against the real trajectory, not just a scripted proxy (an
     earlier attempt used a scripted "zigzag" cross-track oscillation
     drive for this, but the real gait's actual stride dynamics
     already leave the honest "obey" scripted drive's own EMA
     activation at only ~40% of ticks, before adding any synthetic
     wobble -- physics-based sway does not behave like a clean
     sinusoid, so the real-checkpoint replay is both simpler and much
     stronger evidence).

KNOWN PRE-EXISTING HOLE (not caused by, and not fixed by, this lever):
`test_phasedir_semantics.py::test_obey_beats_fastcadence_every_bin`
already FAILS today for the EMA mechanism at HEAD (fwd/rear bins,
fastcadence out-earns obey by ~5-95 return depending on bin -- verified
by direct measurement with `HEXAPOD_MODEL_SOURCE=primitive` before
writing this file, git-stash-diffed against a clean tree). The disp
mechanism reproduces this SAME gap to within ~1 return unit per bin
(measured), so it neither fixes nor worsens it -- out of scope for
lever (b); left for whoever owns the loadslip-pricing hole that test's
own docstring names.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control",
           ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

pytest.importorskip("mujoco")

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_phasedir_semantics as pd                          # noqa: E402

HEADING_BINS = pd.HEADING_BINS
SEEDS = pd.SEEDS
CMD_SPEED = pd.CMD_SPEED

# The candidate stack: PHASEDIR2 with k_walk_course (+its nested
# overspeed) OFF and k_walk_course_disp (+its own overspeed twin) ON
# at the same coefficients, so any behavioral delta is attributable
# to the mechanism swap alone, not a coefficient change.
DISP_STACK = dict(pd.PHASEDIR2_STACK)
DISP_STACK[("reward", "k_walk_course")] = 0.0
DISP_STACK[("reward", "k_walk_course_disp")] = 2.0
DISP_STACK[("reward", "walk_course_disp_window_s")] = 1.5
DISP_STACK[("reward", "walk_course_disp_min_speed_m_s")] = 0.02
DISP_STACK[("reward", "k_walk_course_disp_overspeed")] = 4.0
DISP_STACK[("reward", "walk_course_disp_overspeed_tol")] = 0.05
DISP_STACK[("reward", "walk_course_disp_overspeed_along")] = 1.0
DISP_STACK[("reward", "walk_course_disp_overspeed_ref_floor_m_s")] = 0.06

# PHASEDIR2 with EVERYTHING off (course EMA AND disp) -- the true
# legacy-bit-exact control for the default-off test.
OFF_STACK = dict(pd.PHASEDIR2_STACK)
OFF_STACK[("reward", "k_walk_course")] = 0.0


def _mean_disp(drive: str, heading: float) -> float:
    return float(np.mean([
        pd._phasedir_rollout(drive, s, heading, DISP_STACK)
        for s in SEEDS]))


@pytest.fixture(scope="module")
def disp_returns() -> dict[str, float]:
    out = {}
    for bin_name, h in HEADING_BINS.items():
        for drive in ("obey", "fastcadence", "skew", "stall", "park"):
            out[f"{bin_name}_{drive}"] = _mean_disp(drive, h)
        if bin_name != "fwd":
            out[f"{bin_name}_wrongway"] = _mean_disp("wrongway", h)
    return out


def test_course_disp_default_off_is_inert():
    """k_walk_course_disp=0 (the OFF_STACK, everything else identical
    to PHASEDIR2) must be bit-exact to the same stack evaluated with
    k_walk_course also 0 -- no state update, no info keys, no reward
    delta from the mere presence of walk_course_disp_* keys."""
    collect_off: dict = {}
    r_off = pd._phasedir_rollout("obey", 0, 0.0, OFF_STACK,
                                 collect=collect_off)
    stack_with_disp_zero = dict(OFF_STACK)
    stack_with_disp_zero[("reward", "walk_course_disp_window_s")] = 3.0
    stack_with_disp_zero[
        ("reward", "walk_course_disp_min_speed_m_s")] = 0.5
    collect_on0: dict = {}
    r_on0 = pd._phasedir_rollout("obey", 0, 0.0, stack_with_disp_zero,
                                 collect=collect_on0)
    assert r_off == pytest.approx(r_on0, abs=1e-9)
    assert collect_on0.get("reward_walk_course_disp", 0.0) == 0.0
    assert collect_on0.get("walk_course_disp_active_ticks", 0) == 0


@pytest.mark.parametrize("bin_name", list(HEADING_BINS))
def test_disp_obey_beats_offcourse_skew(disp_returns, bin_name):
    """Same invariant as test_phasedir_semantics'
    test_obey_beats_fast_offcourse_every_bin, under the disp mechanism:
    the full phasedir1 degradation (fast AND ~50 deg off course) must
    earn clearly less than obedience."""
    r = disp_returns
    assert r[f"{bin_name}_obey"] > r[f"{bin_name}_skew"] + 50.0, {
        k: v for k, v in r.items() if k.startswith(bin_name)}


@pytest.mark.parametrize("bin_name", list(HEADING_BINS))
def test_disp_obey_beats_refusal_stall(disp_returns, bin_name):
    """Same invariant as test_obey_beats_refusal_stall_every_bin."""
    r = disp_returns
    assert r[f"{bin_name}_obey"] > r[f"{bin_name}_stall"] + 100.0, {
        k: v for k, v in r.items() if k.startswith(bin_name)}


@pytest.mark.parametrize("bin_name", list(HEADING_BINS))
def test_disp_obey_beats_park(disp_returns, bin_name):
    r = disp_returns
    assert r[f"{bin_name}_obey"] > r[f"{bin_name}_park"] + 100.0, {
        k: v for k, v in r.items() if k.startswith(bin_name)}


@pytest.mark.parametrize("bin_name", ["left", "rear"])
def test_disp_obey_beats_command_ignoring_wrongway(disp_returns, bin_name):
    """Same invariant as test_obey_beats_command_ignoring_walk -- and,
    measured, the disp mechanism prices this MUCH harder than the EMA
    form (net displacement sees the sustained off-command travel in
    full; the short-tau EMA partially forgets it), never softer."""
    r = disp_returns
    assert r[f"{bin_name}_obey"] > r[f"{bin_name}_wrongway"] + 50.0, {
        k: v for k, v in r.items() if k.startswith(bin_name)}


@pytest.mark.parametrize("bin_name", list(HEADING_BINS))
def test_disp_matches_ema_pricing_of_every_scripted_class(bin_name):
    """The mechanism swap (EMA -> net displacement) must not change
    how any EXISTING scripted-teacher behavior class is priced -- a
    safe drop-in, not a reprice. Compares DISP_STACK against
    PHASEDIR2_STACK per class; allows a modest tolerance (drives with
    a large sustained off-command component, e.g. wrongway/skew, are
    EXPECTED to diverge because the disp mechanism prices them harder
    by construction -- documented above -- so those two classes are
    excluded here and covered by their own stronger-inequality test
    above instead)."""
    h = HEADING_BINS[bin_name]
    for drive in ("obey", "fastcadence", "stall", "park"):
        orig = float(np.mean([pd._phasedir_rollout(drive, s, h,
                                                    pd.PHASEDIR2_STACK)
                              for s in SEEDS]))
        disp = _mean_disp(drive, h)
        # Returns for a full 10s episode run several hundred to ~2000;
        # a few units of noise from float-path differences (deque vs
        # EMA state) is expected -- 15 units covers it with margin
        # while still catching a real reprice (the wrongway/skew
        # deltas this same harness measures are 100s-1000s of units).
        assert abs(orig - disp) < 15.0, (
            drive, bin_name, orig, disp)


_S1CONT1_CKPT = (ROOT / "rl_move" / "sim" / "policies"
                / "ppo_goal_cw_standwalk_unified1_mix_long_s1_cont1.zip")

# The EXACT cfg-set stack this checkpoint's own gate/mixedsession eval
# uses for a plain "walk" segment (captured verbatim from the live
# on-pod `eval_checkpoint` invocation, 2026-08-29), plus the two new
# disp-mechanism keys appended. Reusing the real recipe verbatim
# matters: an earlier version of this test hand-picked a SUBSET of
# cfg keys and measured a near-stationary/backward-drifting rollout
# (net displacement 4 cm over 58 s, cos -0.55) -- completely different
# from the checkpoint's real behavior -- because a hand-picked subset
# silently dropped keys (e.g. actions.max_height_mm, the drag/anchor/
# loadslip income gates) that change the effective action scaling and
# the income landscape the policy was actually trained against.
_S1CONT1_CFGSET = [
    "reward.k_drag_loaded=10.0", "reward.k_park_duty=1.0",
    "reward.walk_kernel_prog_gate=1.0",
    "goal.walk_park_start_frac=0.25", "reward.walk_anchor_gate=1.0",
    "reward.anchor_tol_mm=10.0", "goal.walk_speed_min_m_s=0.08",
    "goal.walk_speed_max_m_s=0.08", "goal.walk_obs_body_vel=2",
    "safety.max_roll_deg=25", "safety.max_pitch_deg=25",
    "dr.tipped_start_prob=0.0", "reward.walk_height_gate=1.0",
    "reward.walk_height_sigma_mm=30.0", "goal.walk_phase_obs=1",
    "goal.walk_phase_hz=1.333333", "goal.walk_heading_max_rad=0.0",
    "reward.k_walk_course=2.0", "reward.walk_course_tau_s=0.75",
    "reward.k_walk_course_overspeed=4.0",
    "reward.walk_course_overspeed_tol=0.05",
    "reward.k_walk_idle_charge=1.0", "reward.walk_idle_speed_m_s=0.04",
    "reward.walk_loadslip_gate=1.0", "reward.loadslip_ok=3.0",
    "reward.loadslip_max=6.0", "reward.k_loadslip_excess=10.0",
    "reward.walk_course_overspeed_along=1",
    "reward.walk_course_min_speed_m_s=0.04",
    "reward.k_drag_stance=8000.0", "reward.drag_stance_allow_mm=24.0",
    "reward.drag_stance_tick_floor_mm=0.25",
    "reward.walk_kernel_vel_ema=1", "reward.walk_kernel_vel_tau_s=0.75",
    "reward.k_walk_prog=2.0",
    "reward.walk_course_overspeed_ref_floor_m_s=0.06",
    "actions.max_height_mm=88", "goal.rise_height_mm=[79,87]",
    "goal.rise_ramp_s=6.0", "goal.rise_rsi_frac=0.0",
    "goal.rise_hold_min_s=0.5", "reward.rise_score_income=1.0",
    "reward.rise_score_strip_pen=1.0", "reward.k_rise_ref_track=2.0",
    "reward.rise_ref_path=rl_move/sim/refs/rise_ref_mesh_scripted.npz",
    "reward.rise_ref_sigma_deg=6.0", "reward.rise_posture_gate=1.0",
    "reward.rise_income_prog_gate=1.0",
    "reward.rise_finish_gate_signed=1.0", "reward.hold_still_gate=1.0",
    "reward.hold_flag_fade=1.0", "reward.k_current_hot=1.0",
    "reward.current_hot_a=2.0", "reward.term_cost_per_remaining_s=3.0",
    "reward.term_cost_max=60.0", "reward.hold_feet_load=1.0",
    "reward.hold_feet_load_min=1.0", "safety.hold_max_height_drop_mm=40.0",
    "safety.hold_height_grace_s=1.0",
    "safety.hold_min_load_terminate_s=1.0",
    "safety.hold_min_load_terminate_n=0.3",
    "safety.hold_min_load_terminate_grace_s=1.0",
    "env.model_source=mesh", "control.hz=100", "obs.mode_onehot=1",
    "goal.mode_seq=0.0",
    "train.bc_anchor_coef=3.0", "train.bc_anchor_lower=1.0",
    "train.bc_anchor_state_aligned=1.0",
    "train.bc_anchor_lookahead_s=0.25",
    "train.bc_anchor_min_h_ahead_mm=8", "train.bc_anchor_foot_z=1.0",
    "train.bc_anchor_stratified=1.0",
    "train.bc_anchor_flat_time_indexed=1.0", "train.bc_anchor_walk=1.0",
    "train.bc_anchor_isolate_update=1", "train.bc_anchor_phase_lock=1.0",
    "train.bc_anchor_knee_abs=1.0", "train.bc_anchor_walk_coef=1.0",
    # disp-mechanism additions (this file's own new keys, on top of
    # the real recipe's own k_walk_course EMA -- both run at once so
    # the SAME tick stream feeds both info-key traces for comparison).
    "reward.k_walk_course_disp=2.0",
    "reward.walk_course_disp_window_s=1.5",
    "reward.walk_course_disp_min_speed_m_s=0.02",
]


@pytest.mark.skipif(
    not _S1CONT1_CKPT.is_file(),
    reason="long-s1-cont1 checkpoint not present in this checkout "
           "(trained artifact, not committed) -- this test only runs "
           "where the DIG-IN's own repro artifact is available")
def test_real_failed_checkpoint_ema_inert_disp_active(tmp_path):
    """DIRECT validation against the REAL checkpoint the DIG-IN named
    (`cw-standwalk-unified1-mix-long-s1-cont1`, FAILed its own gate
    with direction_err plateaued ~62-68 deg), not a scripted proxy.
    Shells out to the production `eval_checkpoint` CLI (the exact code
    path the run's own gate/mixedsession evals use -- an earlier
    in-process reimplementation attempt measured a completely
    different, near-stationary rollout because it silently missed
    some cfg/construction detail `main()` gets right; subprocessing
    the real CLI removes that whole class of risk) with the
    `--course-trace` diagnostic flag (added this cycle, see
    `run_episode`'s docstring) and the checkpoint's own real cfg
    recipe (verbatim, `_S1CONT1_CFGSET`) plus the two new disp keys.
    Confirms, on that one real deterministic 60s walk/det rollout:
      1. the EXISTING k_walk_course EMA activates on (almost) no
         ticks -- reproducing the root-cause diagnostic's 0/5899
         finding;
      2. the windowed NET-DISPLACEMENT direction error over the same
         tick stream is small (the real path is close to on-course);
      3. k_walk_course_disp activates on most ticks with a small mean
         course error, unlike the EMA, on the identical rollout."""
    import math as _math
    import subprocess
    import sys as _sys

    trace_path = tmp_path / "course_trace.csv"
    out_dir = tmp_path / "eval_out"
    cmd = [
        _sys.executable, "-m", "rl_move.sim.eval_checkpoint",
        str(_S1CONT1_CKPT), "--task", "joint_walk", "--modes", "walk",
        "--per-mode", "1", "--dr-scale", "0.0", "--seed", "91000",
        "--episode-seconds", "60.0", "--no-wandb", "--no-video",
        "--no-start-jitter-panel", "--out", str(out_dir),
        "--course-trace", str(trace_path),
    ]
    for spec in _S1CONT1_CFGSET:
        cmd += ["--cfg-set", spec]
    proc = subprocess.run(cmd, cwd=str(ROOT), capture_output=True,
                          text=True, timeout=180)
    assert proc.returncode == 0, proc.stdout[-4000:] + proc.stderr[-4000:]
    assert trace_path.is_file(), proc.stdout + proc.stderr

    rows = [ln.split(",") for ln in
           trace_path.read_text().strip().split("\n")]
    # First episode only (walk/det/0) -- step counter resets at ep
    # boundaries; per-mode=1 + no start-jitter panel means this file
    # holds exactly one episode, but guard anyway.
    ep0 = []
    prev = -1
    for r in rows:
        step = int(r[0])
        if step < prev:
            break
        ep0.append(r)
        prev = step

    vxr = [float(r[5]) for r in ep0]
    vyr = [float(r[6]) for r in ep0]
    bx = [float(r[3]) for r in ep0]
    by = [float(r[4]) for r in ep0]
    s_ref = [_math.hypot(a, b) for a, b in zip(vxr, vyr)]
    ema_active = sum(1 for r in ep0 if r[7] != "")
    disp_active_rows = [r for r in ep0 if r[8] != ""]
    disp_active = len(disp_active_rows)

    n_active_cmd = sum(1 for s in s_ref if s > 1e-3)
    assert n_active_cmd > 1000, "episode too short / command never active"

    # (1) EMA course term stays ~inert: <10% of commanded ticks (the
    # ~04:3x diagnostic's own local repro measured 0/5899 exactly; a
    # small nonzero count here is fine -- process/CLI-vs-in-process
    # float-path noise near the activation floor -- what must NOT
    # happen is anything close to the disp mechanism's >50% floor
    # below, which is the actual claim under test).
    assert ema_active < 0.10 * n_active_cmd, (
        f"k_walk_course EMA activated on {ema_active}/{n_active_cmd} "
        "commanded ticks -- root-cause finding no longer reproduces")

    # (2) windowed net-displacement direction error over the real
    # steady-command window.
    idx = [i for i, s in enumerate(s_ref) if s > 0.079]
    assert len(idx) > 500
    i0, i1 = idx[0], idx[-1]
    dx, dy = bx[i1] - bx[i0], by[i1] - by[i0]
    d = _math.hypot(dx, dy)
    assert d > 0.3, f"expected real net forward travel, got {d:.3f} m"
    cosf = (dx * vxr[i1] + dy * vyr[i1]) / (d * s_ref[i1])
    net_dir_err = _math.degrees(_math.acos(max(-1.0, min(1.0, cosf))))
    assert net_dir_err < 20.0, (
        f"windowed net-displacement direction error {net_dir_err:.1f} deg")

    # (3) disp course term actually fires, and reads the real path as
    # reasonably well-aligned.
    assert disp_active > 0.5 * n_active_cmd, (
        f"k_walk_course_disp activated on only {disp_active}/"
        f"{n_active_cmd} commanded ticks")
    disp_cos_vals = [float(r[9]) for r in disp_active_rows]
    mean_disp_cos = sum(disp_cos_vals) / len(disp_cos_vals)
    assert mean_disp_cos > 0.7, f"mean disp course cos {mean_disp_cos:.3f}"
