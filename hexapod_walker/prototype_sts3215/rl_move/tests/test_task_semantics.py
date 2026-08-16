"""MDP_PREFLIGHT — task-semantics tests (binding gate, operator 08-10).

PPO must never be the discovery mechanism for a reward/eval bug. For
each task mode we keep a tiny bank of hand-constructed or previously
observed trajectories — correct behavior, partial success, known
exploits, refusal (freeze), unsafe behavior — and mechanically assert
the required ordering with useful margins, under the FULL reward
stack the arm will train with (RESEARCH_RULES.md "MDP_PREFLIGHT"; the
launcher requires `--evidence` naming the passing run of this suite).
Required orderings per mode (GPT bootstrap doc + operator, 08-10):

    RISE:  honest six-foot plant > partial honest rise >
           flag-leg/tripod-at-height > freeze > unsafe thrash
    LOWER: honest lower/sit > partial descent > full-height refusal >
           flag-leg/outrigger cheat > unsafe behavior
    TURN:  commanded yaw, correct direction > partial yaw >
           fixed natural drift / straight walking > parking
    WALK:  useful commanded progress > march-in-place/paddle stall >
           park/refusal   (physical foot slip alone is NOT failure)

History that motivates each bank lives in the test docstrings. These
are sim-rollout tests (~minutes, CPU); run them on demand, not as part
of a quick unit sweep:

    ../../.venv/bin/python -m pytest rl_move/tests/test_task_semantics.py -v

Add a bank whenever a new exploit is seen on video: encode the exploit
as a scripted trajectory here FIRST, then fix the reward, then prove
the ordering. Never fix a reward without pinning the exploit in a test.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

pytest.importorskip("mujoco")

from rl_move.config import load_config  # noqa: E402
from rl_move.robot_state import DEG2RAD  # noqa: E402
from rl_move.sim.joint_task import (  # noqa: E402
    SimHexapodJointGoalEnv, q_rad_to_action)
from rl_move.sim.servo_model import SimServoParams  # noqa: E402

# --------------------------------------------------------------------------
# RISE bank — belly -> walkable plant stance (+111 mm), full shaping stack.
#
# Exploits pinned here (all video-confirmed on real runs):
#   freeze  the paid-plateau exploit (arrival-gate sign bug, cw-uni-rfix-*)
#   stilt   hip 0 / knee 80 tip-toe pop — gamed rise "6/6" in rfix-fresh1
#           and reappeared in cw-stand-b2p1 as the flag-leg/tripod cheat.

RISE_REF = "rl_move/sim/refs/rise_ref_belly2plant.npz"
RISE_OVERRIDES = {
    ("actions", "max_height_mm"): 115.0,
    ("goal", "rise_height_mm"): [108.0, 114.0],
    ("goal", "rise_ramp_s"): 6.0,
    ("reward", "k_rise_ref_track"): 2.0,
    ("reward", "rise_ref_path"): RISE_REF,
    ("reward", "rise_posture_gate"): 1.0,
    ("reward", "rise_income_prog_gate"): 1.0,
    ("reward", "rise_finish_gate_signed"): 1.0,
}
SEEDS = (0, 1, 2)


def _make_rise_env(seed: int, overrides=None) -> SimHexapodJointGoalEnv:
    cfg = load_config()
    for (sec, leaf), val in (overrides or RISE_OVERRIDES).items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointGoalEnv(
        params=SimServoParams.from_cfg(None), randomize=False,
        dr_scale=0.0, episode_seconds=16.0, seed=seed, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "rise" else 0.0)
    gen.force_rise_start = "flat"
    return env


def _rise_rollout(policy: str, seed: int, overrides=None) -> dict:
    env = _make_rise_env(seed, overrides)
    env.reset()
    ref = np.load(ROOT / RISE_REF)
    q_ref, ramp_ref = ref["q_rad"], int(ref["ramp_i0"])
    q0 = env.data.qpos[env._qadr].copy()
    q_stilt = np.array([0.0, 0.0, 80.0] * 6) * DEG2RAD
    # partial = follow the reference only halfway up, then hold that
    # honest feet-down mid-rise pose (partial success, not a cheat).
    j_half = ramp_ref + (len(q_ref) - ramp_ref) // 2
    rng = np.random.default_rng(seed)

    total, step = 0.0, 0
    while True:
        if policy == "replay":
            j = ramp_ref + (step - env._rise_ramp_i0)
            act = q_rad_to_action(q_ref[min(max(j, 0), len(q_ref) - 1)])
        elif policy == "partial":
            j = min(ramp_ref + (step - env._rise_ramp_i0), j_half)
            act = q_rad_to_action(q_ref[min(max(j, 0), len(q_ref) - 1)])
        elif policy == "flagleg":
            # The cw-stand-b2p1 / cw-stand-plantgate1 video cheat: five
            # legs execute the honest rise, one leg stays flagged
            # straight out and never takes load.
            j = ramp_ref + (step - env._rise_ramp_i0)
            q = q_ref[min(max(j, 0), len(q_ref) - 1)].copy()
            q[0:3] = q0[0:3]
            act = q_rad_to_action(q)
        elif policy == "freeze":
            act = q_rad_to_action(q0)
        elif policy == "thrash":
            act = rng.uniform(-1.0, 1.0, size=18)
        else:  # stilt
            act = q_rad_to_action(
                q0 if step < env._rise_ramp_i0 else q_stilt)
        _obs, r, term, trunc, _info = env.step(act)
        total += float(r)
        step += 1
        if term or trunc:
            terminated = term
            break
    # End-state PLANT_SPEC check (the shared valid-plant criterion —
    # sim_env.valid_plant via env.plant_report). A terminated episode
    # never ends in a valid stand.
    h_err = (float(env.data.xpos[env._chassis_bid, 2]) - env._z0
             - env._h_target)
    plant_ok, detail = ((False, {"terminated": True}) if terminated
                        else env.plant_report(height_err_m=h_err))
    env.close()
    return {"ret": total, "plant": bool(plant_ok), "detail": detail}


@pytest.fixture(scope="module")
def rise_bank() -> dict[str, list[dict]]:
    """Per-seed rollouts for each bank policy (rolled out once)."""
    return {p: [_rise_rollout(p, s) for s in SEEDS]
            for p in ("replay", "partial", "freeze", "stilt", "thrash")}


@pytest.fixture(scope="module")
def rise_returns(rise_bank) -> dict[str, float]:
    """Mean return over seeds for each bank policy."""
    return {p: float(np.mean([r["ret"] for r in rolls]))
            for p, rolls in rise_bank.items()}


def test_rise_correct_dominates_known_exploits(rise_returns):
    """The demonstrated belly->plant path must out-earn BOTH measured
    degenerate strategies by a wide margin (>=2x and +50 absolute) —
    the +952 / +225 / -195 smoke of 08-10, made binding."""
    replay = rise_returns["replay"]
    best_cheat = max(rise_returns["freeze"], rise_returns["stilt"])
    assert replay > 2.0 * best_cheat and replay > best_cheat + 50.0, (
        f"reward stack prefers a known cheat: {rise_returns} — fix the "
        "reward BEFORE launching any rise arm (PPO will find this).")


def test_rise_freeze_is_net_negative(rise_returns):
    """Not-trying must be net NEGATIVE income, by construction — the
    paid freeze plateau (arrival-gate sign bug) can never return."""
    assert rise_returns["freeze"] < 0.0, (
        f"freezing earns {rise_returns['freeze']:+.1f} — the reward pays "
        "for doing nothing again.")


def test_rise_partial_honest_beats_cheat_and_refusal(rise_returns):
    """An honest half-rise (feet down, stopped mid-reference) must
    out-earn both the stilt cheat and refusal, and lose to the full
    rise — otherwise the gradient toward trying honestly is broken.
    (Freeze-vs-thrash is deliberately NOT asserted: early safety
    termination truncates penalty accrual, and what training needs is
    honest > everything else, which IS asserted.)"""
    partial = rise_returns["partial"]
    assert rise_returns["replay"] > partial, rise_returns
    assert partial > rise_returns["stilt"], (
        f"a known cheat out-earns honest partial progress: {rise_returns}")
    assert partial > rise_returns["freeze"], rise_returns
    assert rise_returns["thrash"] < partial, (
        f"unsafe thrash rivals honest progress: {rise_returns}")


def test_rise_valid_plant_separates_stand_from_cheats(rise_bank):
    """The GEOMETRIC stand criterion (PLANT_SPEC, operator 2026-08-10):
    height + attitude + feet down + no flag legs + CoM inside the
    down-feet support polygon + walkable footprint. The demonstrated
    belly->plant path must END in a valid plant on every seed; the
    stilt pop, the freeze, and the honest-but-incomplete half rise
    must all FAIL it (that is the whole point — torso height alone
    never defines a stand)."""
    for r in rise_bank["replay"]:
        assert r["plant"], (
            f"the demonstrated rise fails its own stand spec: "
            f"{r['detail']} — fix PLANT_SPEC, not the reference.")
    for p in ("stilt", "freeze", "partial"):
        for r in rise_bank[p]:
            assert not r["plant"], (
                f"'{p}' passes the valid-plant spec: {r['detail']} — "
                f"the geometric criterion has a hole.")


# --------------------------------------------------------------------------
# RISE bank, STAND-SCORE stack (cw-stand-score1 lineage, operator 08-10
# evening). cw-stand-plantgate1 proved multiplicative gates LEAK: the
# flag-leg cheat still collected ~60% of the height income and won even
# warm-started from the honest champion. reward.rise_score_income=1
# zeroes the height income streams and pays ONLY a progress ratchet +
# post-ramp hold on the stand-score S (height x feet-down^2 x hard
# no-flag x plant geometry) — so the cheat's income is ~0 by
# construction, not a discounted consolation. No reference tracking:
# the score must carry the ordering on its own.

SCORE_OVERRIDES = {
    ("actions", "max_height_mm"): 115.0,
    ("goal", "rise_height_mm"): [108.0, 114.0],
    ("goal", "rise_ramp_s"): 6.0,
    ("reward", "rise_score_income"): 1.0,
    # 08-11 (post-rsi2): strip the legacy k_height PENALTY too — it
    # funded the flag-leg cheat (belly rest -1.2/tick vs cheat's
    # -0.5/tick rent, so the cheat was the reachable optimum). With it
    # gone, height gradient comes only from score/ref income.
    ("reward", "rise_score_strip_pen"): 1.0,
    # Reference tracking rides along since cw-stand-scoreref1
    # (08-10 late): cw-stand-score1 proved the score income alone is
    # unfindable — env/rise_score sat at ~0.01 for 2M steps because the
    # conjunction S is near-zero everywhere but the actual stand, so
    # the ratchet never fired (exploration hole, not a pricing hole).
    # The reference is the exploration crutch (bank replay reaches the
    # stand and earns dominantly); the score stays the only endpoint
    # payer, so tracking cannot be satisfied by a cheat. Anneal the
    # track weight across arms; the score income is what remains.
    ("reward", "k_rise_ref_track"): 2.0,
    ("reward", "rise_ref_path"): RISE_REF,
    # Tight kernel (default 12 -> 6 deg): at 12 the crutch alone paid
    # stilt +155 and flag-leg +288 per episode (measured, seed 0) —
    # near-miss poses farm a wide kernel. At 6 deg the demonstration
    # keeps ~full pay (its own RMS ~0) while the cheats drop to scraps
    # (the grounded-feet gate takes the airborne share on top).
    ("reward", "rise_ref_sigma_deg"): 6.0,
    # The solved lower-line fixes ride along in the real run cfg; under
    # score mode they are no-ops for rise by construction (they scale
    # streams the score switch zeroes) — bank under the EXACT stack.
    ("reward", "rise_posture_gate"): 1.0,
    ("reward", "rise_income_prog_gate"): 1.0,
    ("reward", "rise_finish_gate_signed"): 1.0,
}

SCORE_POLICIES = ("replay", "partial", "freeze", "stilt", "flagleg",
                  "thrash")


@pytest.fixture(scope="module")
def score_bank() -> dict[str, list[dict]]:
    return {p: [_rise_rollout(p, s, SCORE_OVERRIDES) for s in SEEDS]
            for p in SCORE_POLICIES}


@pytest.fixture(scope="module")
def score_returns(score_bank) -> dict[str, float]:
    return {p: float(np.mean([r["ret"] for r in rolls]))
            for p, rolls in score_bank.items()}


def test_score_correct_dominates_all_cheats(score_returns):
    """Same dominance margin as the legacy bank, but now the flag-leg
    cheat — the one that actually beat the gates in training — is in
    the comparison set."""
    replay = score_returns["replay"]
    best_cheat = max(score_returns["freeze"], score_returns["stilt"],
                     score_returns["flagleg"])
    assert replay > 2.0 * best_cheat and replay > best_cheat + 50.0, (
        f"stand-score stack prefers a known cheat: {score_returns}")


def test_score_flagleg_earns_scraps(score_returns):
    """The point of the whole redesign: torso-at-height with a flagged
    leg must earn approximately NOTHING (hard no-flag zero), not a 60%
    consolation, and must lose to honest partial progress."""
    assert score_returns["flagleg"] < 0.1 * score_returns["replay"], (
        f"flag-leg still collects real income: {score_returns}")
    assert score_returns["flagleg"] < score_returns["partial"], (
        f"flag-leg out-earns honest partial rise: {score_returns}")


def test_score_freeze_net_negative(score_returns):
    assert score_returns["freeze"] < 0.0, (
        f"freezing earns {score_returns['freeze']:+.1f} under the "
        "stand-score stack — refusal is being paid again.")


def test_score_honest_ordering(score_returns):
    """replay > partial > every cheat and refusal — with the tight
    (6 deg) crutch kernel the honest half-path's tracking income
    restores the full legacy ordering, stilt included."""
    assert score_returns["replay"] > score_returns["partial"], score_returns
    for p in ("freeze", "stilt", "flagleg", "thrash"):
        assert score_returns["partial"] > score_returns[p], (
            f"'{p}' out-earns honest partial progress: {score_returns}")


def test_score_stilt_is_not_a_paid_plateau(score_returns):
    """The stilt pop earned +225/ep under height income (rfix-fresh1's
    gamed 6/6). Under score income + gated crutch it may keep the
    small grounded early-phase tracking pay (everyone starts AT the
    reference's start pose — that is not a plateau), but parking there
    must trail the honest rise by a wide margin and earn less than a
    quarter of it."""
    assert score_returns["replay"] > score_returns["stilt"] + 50.0, (
        f"stilt is competitive with the honest rise: {score_returns}")
    assert score_returns["stilt"] < 0.25 * score_returns["replay"], (
        f"stilt keeps a real fraction of the honest pay: {score_returns}")


def test_score_replay_ends_in_valid_plant(score_bank):
    """The demonstrated rise must still end PLANT_SPEC-valid under the
    score stack (reward changes must not have bent the sim), and every
    cheat must fail the spec."""
    for r in score_bank["replay"]:
        assert r["plant"], f"replay fails PLANT_SPEC: {r['detail']}"
    for p in ("stilt", "freeze", "partial", "flagleg"):
        for r in score_bank[p]:
            assert not r["plant"], (
                f"'{p}' passes the valid-plant spec: {r['detail']}")


# --------------------------------------------------------------------------
# LOWER bank — standing plant -> commanded descent, under the DEPLOYED
# stance-specialist stack (cw-stand-holdbc1-hard1 cfg-sets, loaded servo
# params). Built 2026-08-11 (was the last owed bank).
#
# Exploits pinned (video-confirmed on real runs):
#   outrig  cw-uni-rfix-postgate1: legs held rigidly aloft as outriggers
#           through the descent — h_err excellent, returns DOUBLED the
#           honest lower before the 60 mm mode-correct allowance fix.
#   aloft   cw-stand-b2p1 family: five legs lower honestly, one leg
#           never comes down (the reverse-handoff eval of 08-11 saw a
#           cosmetic 62-99 mm dangling-foot residue of exactly this on
#           the deployed specialist's learned lower).
#
# The honest demonstration tracks the commanded ramp with all feet
# planted: FixedFootBodyIK anchored at the SETTLED stance (not the
# ideal plant — loaded servos sag ~16 mm), body offset following the
# goal height schedule. Measured returns (3 seeds, 08-11):
#   honest 540 > aloft 461 (85%) > outrig 383 (71%) > partial 103-182
#   > refuse -2..-51 > thrash -77..-107.
# KNOWN THIN MARGIN, documented by the strict-xfail test at the bottom:
# rise_posture_gate prices a lifted leg at pf=5/6 of income, so the
# aloft cheat keeps 85% of the honest return — the reward PREFERS
# honest on every seed (asserted) but does not make cheating a losing
# path vs refusal (the skip-note ideal). This exactly predicts the
# dangling-foot residue seen on the deployed specialist. Any arm whose
# MECHANISM targets lower ticks (e.g. BC anchor on lower) should
# strengthen this margin and flip the xfail.

LOWER_OVERRIDES = {
    ("actions", "max_height_mm"): 115.0,
    ("goal", "rise_height_mm"): [108.0, 114.0],
    ("goal", "rise_ramp_s"): 6.0,
    ("reward", "rise_score_income"): 1.0,
    ("reward", "rise_score_strip_pen"): 1.0,
    ("reward", "k_rise_ref_track"): 2.0,
    ("reward", "rise_ref_path"): RISE_REF,
    ("reward", "rise_ref_sigma_deg"): 6.0,
    ("reward", "rise_posture_gate"): 1.0,
    ("reward", "rise_income_prog_gate"): 1.0,
    ("reward", "rise_finish_gate_signed"): 1.0,
    ("reward", "hold_still_gate"): 1.0,
    ("reward", "hold_flag_fade"): 1.0,
    ("bus", "servo_params"): "loaded",
}
# One leg pointed skyward (hip at its -80 deg limit): pad ends ~300 mm
# above its grounded reference — unambiguously aloft, same behavior
# class as the measured 33-109 mm cheats (which used gentler angles).
_LOWER_LIFT_RAD = np.array([0.0, -80.0, 0.0] * 6) * DEG2RAD
_LOWER_CLEAR_OK_MM = 60.0    # harness posture-strict lower allowance
_LOWER_HERR_OK_MM = 15.0     # harness lower success rule


def _make_lower_env(seed: int) -> SimHexapodJointGoalEnv:
    cfg = load_config()
    for (sec, leaf), val in LOWER_OVERRIDES.items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointGoalEnv(
        params=SimServoParams.from_cfg(cfg), randomize=False,
        dr_scale=0.0, episode_seconds=15.0, seed=seed, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "lower" else 0.0)
    return env


def _lower_rollout(policy: str, seed: int) -> dict:
    from rl_move.body_ik import BodyOffset, FixedFootBodyIK
    env = _make_lower_env(seed)
    env.reset()
    q0 = env.data.qpos[env._qadr].copy()
    h = np.asarray(env._goal_traj.height, dtype=float)
    target = float(env._h_target)          # negative (m), vs settled z0
    ik = FixedFootBodyIK()
    ik.reset(q0)                           # anchor feet at settled stance
    res = ik.solve(BodyOffset(height=target))
    q_low = res.q_rad if res.ok else q0
    rng = np.random.default_rng(seed)
    pads = list(env._pad_bids)
    pad_hist = []
    total, step, terminated = 0.0, 0, False
    while True:
        frac = h[min(step, len(h) - 1)] / target if target else 0.0
        q = q0 + (q_low - q0) * frac
        if policy == "partial":
            q = q0 + (q_low - q0) * min(frac, 0.5)
        elif policy == "refuse":
            q = q0.copy()
        elif policy in ("outrig", "aloft"):
            for leg in ((3, 5) if policy == "outrig" else (0,)):
                sl = slice(3 * leg, 3 * leg + 3)
                q[sl] = q0[sl] + (_LOWER_LIFT_RAD[sl] - q0[sl]) * frac
        elif policy == "thrash":
            q = None
        act = rng.uniform(-1.0, 1.0, 18) if q is None else q_rad_to_action(q)
        _obs, r, term, trunc, _info = env.step(act)
        total += float(r)
        step += 1
        pad_hist.append([float(env.data.xpos[b, 2]) for b in pads])
        if term or trunc:
            terminated = bool(term)
            break
    tail = max(1, int(round(0.5 / env.dt)))   # harness 0.5 s tail mean
    clear_mm = ((np.asarray(pad_hist[-tail:]).mean(axis=0)
                 - env._pad_z_ref) * 1000.0)
    h_err_mm = (float(env.data.xpos[env._chassis_bid, 2])
                - env._z0 - target) * 1000.0
    env.close()
    return {"ret": total, "terminated": terminated,
            "h_err_mm": float(h_err_mm),
            "clear_mm": [float(c) for c in clear_mm]}


@pytest.fixture(scope="module")
def lower_bank() -> dict[str, list[dict]]:
    return {p: [_lower_rollout(p, s) for s in SEEDS]
            for p in ("honest", "partial", "refuse", "outrig", "aloft",
                      "thrash")}


@pytest.fixture(scope="module")
def lower_returns(lower_bank) -> dict[str, float]:
    return {p: float(np.mean([r["ret"] for r in rolls]))
            for p, rolls in lower_bank.items()}


def test_lower_correct_beats_outrigger_cheat(lower_bank, lower_returns):
    """Honest command-tracking descent must out-earn BOTH measured leg-
    aloft strategies on EVERY seed, with at least the pf=5/6 posture-
    gate differential intact (>=1.10x and +50 absolute on the mean) —
    the postgate1 regression (cheat DOUBLING honest income) can never
    silently return."""
    honest = lower_returns["honest"]
    for p in ("outrig", "aloft"):
        for h_roll, c_roll in zip(lower_bank["honest"], lower_bank[p]):
            assert h_roll["ret"] > c_roll["ret"], (
                f"'{p}' out-earns the honest lower on a seed: "
                f"{h_roll['ret']:.1f} vs {c_roll['ret']:.1f} — fix the "
                "reward BEFORE launching any lower arm.")
        assert honest > 1.10 * lower_returns[p] and \
            honest > lower_returns[p] + 50.0, (
                f"honest-vs-'{p}' margin collapsed: {lower_returns}")


def test_lower_honest_gradient(lower_returns):
    """Full descent > half descent > full-height refusal, refusal is
    net NEGATIVE (not-trying is never a paid path), unsafe thrash is
    below honest partial progress."""
    assert lower_returns["honest"] > lower_returns["partial"] \
        > lower_returns["refuse"], lower_returns
    assert lower_returns["refuse"] < 0.0, (
        f"refusing to lower earns {lower_returns['refuse']:+.1f}")
    assert lower_returns["thrash"] < lower_returns["partial"], lower_returns


def test_lower_end_posture_separates_cheats(lower_bank):
    """The harness posture-strict criterion (|h_err|<=15 mm, all pads
    <=60 mm over the 0.5 s tail) must accept the honest descent and
    reject both aloft cheats on every seed — the eval-side guarantee
    that a cheat can never grade as a clean lower."""
    for r in lower_bank["honest"]:
        assert not r["terminated"], r
        assert abs(r["h_err_mm"]) <= _LOWER_HERR_OK_MM, r
        assert max(r["clear_mm"]) <= _LOWER_CLEAR_OK_MM, r
    for p in ("outrig", "aloft"):
        for r in lower_bank[p]:
            assert max(r["clear_mm"]) > _LOWER_CLEAR_OK_MM, (
                f"'{p}' passes the posture-strict lower spec: {r}")


@pytest.mark.xfail(
    strict=True,
    reason="KNOWN pricing gap (08-11): rise_posture_gate prices a "
           "lifted leg at pf=5/6 of income, so the one-leg-aloft lower "
           "keeps ~85% of the honest return — cheating out-earns "
           "refusal by a wide margin. This is the incentive behind the "
           "deployed specialist's cosmetic 62-99 mm dangling foot "
           "(eval_handoff_reverse, 08-11). Strengthen the pricing (or "
           "BC-anchor lower ticks) before the next lower-MECHANISM "
           "arm; a fix flips this to strict-XPASS and must remove the "
           "marker.")
def test_lower_cheat_is_not_a_paid_path(lower_returns):
    assert max(lower_returns["outrig"], lower_returns["aloft"]) \
        < lower_returns["refuse"]


# --------------------------------------------------------------------------
# Shared walk-lineage champion reward config (cw-walk-longdist lineage
# cfg-sets) — the stack WALK and TURN banks run under.

WALK_OVERRIDES = {
    ("reward", "k_step_event"): 1.0,
    ("reward", "k_drag_loaded"): 10.0,
    ("reward", "k_park_duty"): 1.0,
    ("reward", "walk_kernel_prog_gate"): 1.0,
    ("reward", "walk_anchor_gate"): 1.0,
    ("reward", "anchor_tol_mm"): 10.0,
    ("goal", "walk_speed_min_m_s"): 0.05,
    ("goal", "walk_speed_max_m_s"): 0.06,
}
WALK_CMD_VX = 0.05      # champion command band; tape ran 0.03/0.05
WALK_PLANT = (20.0, 80.0)


# --------------------------------------------------------------------------
# TURN bank — commanded turn-in-place under the full yaw stack a turn
# arm would train with (kernel + achieved-rotation gate + the 08-10
# anti-drift terms k_yaw_prog / k_yaw_still). The failure this bank
# pins: yawcmd1/yawgate1/yawgate2 all learned a command-invariant
# ~+0.09 rad/s left drift because the symmetric kernel never charges
# wrong-direction rotation. The scripted gait turns BOTH directions on
# the real robot (CURRENT_TRUTHS) — it is the honest reference here.

TURN_OVERRIDES = dict(WALK_OVERRIDES)
TURN_OVERRIDES.update({
    ("goal", "walk_yaw_cmd"): 1.0,
    ("reward", "k_walk_yaw"): 1.0,
    ("reward", "walk_yaw_kernel_gate"): 1.0,
    ("reward", "k_yaw_prog"): 1.0,
    ("reward", "k_yaw_still"): 50.0,
    # 08-11 latent-defect fixes (probe_walk_income; see the
    # stillness-subsidy bank below). Any turn arm MUST train with both:
    # heading-hold yaw income gated on achieved linear progress, and
    # the drift charge priced on the wz EMA (DC drift) instead of the
    # honest gait's zero-mean stride oscillation.
    ("reward", "walk_yaw_hold_prog_gate"): 1.0,
    ("reward", "yaw_still_avg_s"): 1.0,
})
# The stack the mirror2 lineage ACTUALLY trained with (pre-fix) — kept
# so the subsidy bank below can prove the defect existed and the fix
# closes it (the hold_legacy pattern).
TURN_LEGACY_OVERRIDES = {
    k: v for k, v in TURN_OVERRIDES.items()
    if k[1] not in ("walk_yaw_hold_prog_gate", "yaw_still_avg_s")}
TURN_CMD_WZ = 0.25       # rad/s, tested at BOTH signs
DRIFT_WZ = 0.09          # the measured structural left drift


def _make_turn_env(seed: int, overrides: dict | None = None):
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv

    cfg = load_config()
    for (sec, leaf), val in (overrides or TURN_OVERRIDES).items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=False,
        dr_scale=0.0, episode_seconds=15.0, seed=seed, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad", "walk"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "walk" else 0.0)
    return env


def _turn_rollout(policy: str, wz_cmd: float, seed: int,
                  overrides: dict | None = None) -> float:
    from tripod_gait import TripodGait

    env = _make_turn_env(seed, overrides)
    env.reset()
    traj = env._goal_traj
    n = len(traj.vx)
    hold_n = ramp_n = int(round(1.0 / env.dt))
    traj.vx[:] = 0.0
    traj.vy[:] = 0.0
    traj.wz[:] = wz_cmd
    traj.wz[:hold_n] = 0.0
    traj.wz[hold_n:hold_n + ramp_n] = np.linspace(0.0, wz_cmd, ramp_n)

    gait = TripodGait(vx=0.0)
    gait.sync_plant_stance(*WALK_PLANT)
    plant_rad = np.array([0.0, *WALK_PLANT] * 6) * DEG2RAD
    gait.reset_phase()

    total, step = 0.0, 0
    while True:
        t = step * env.dt
        cmd = float(traj.wz[min(step, n - 1)])
        if policy == "turn":          # correct direction, full command
            gait.set_velocity(omega=cmd)
            act = q_rad_to_action(np.asarray(gait.desired_deg(t)) * DEG2RAD)
        elif policy == "partial":     # correct direction, ~35% of it
            gait.set_velocity(omega=0.35 * cmd)
            act = q_rad_to_action(np.asarray(gait.desired_deg(t)) * DEG2RAD)
        elif policy == "drift":       # the structural fixed left drift
            gait.set_velocity(omega=DRIFT_WZ if step >= hold_n else 0.0)
            act = q_rad_to_action(np.asarray(gait.desired_deg(t)) * DEG2RAD)
        else:                         # park: ignore the command
            act = q_rad_to_action(plant_rad)
        _obs, r, term, trunc, _info = env.step(act)
        total += float(r)
        step += 1
        if term or trunc:
            break
    env.close()
    return total


@pytest.fixture(scope="module")
def turn_returns() -> dict[str, float]:
    """Mean return over seeds AND both command signs per policy."""
    return {p: float(np.mean([_turn_rollout(p, s_wz * TURN_CMD_WZ, s)
                              for s in SEEDS for s_wz in (+1.0, -1.0)]))
            for p in ("turn", "partial", "drift", "park")}


def test_turn_reward_separates_command_from_drift(turn_returns):
    """Required ordering (module docstring): commanded yaw in the
    correct direction > partial yaw > the fixed structural drift >
    parking. Averaged over BOTH command signs, the fixed drift tracks
    ~0 of the command by symmetry — if it still rivals honest turning,
    the stack cannot punish the exact policy PPO found three times."""
    t = turn_returns
    assert t["turn"] > t["partial"], t
    assert t["partial"] > t["drift"], (
        f"the structural drift out-earns honest partial turning: {t}")
    assert t["drift"] > t["park"], t


def test_turn_command_signs_priced_symmetrically():
    """The SAME correct-direction policy must earn comparably for CW
    and CCW commands — an asymmetric stack would train the drift back
    in. (Also a live check on the open sim-vs-hardware yaw sign audit:
    a sign flip anywhere in the chain shows up here as one direction
    earning like 'drift' instead of like 'turn'.)"""
    ccw = float(np.mean([_turn_rollout("turn", +TURN_CMD_WZ, s)
                         for s in SEEDS]))
    cw = float(np.mean([_turn_rollout("turn", -TURN_CMD_WZ, s)
                        for s in SEEDS]))
    lo, hi = min(ccw, cw), max(ccw, cw)
    assert lo > 0.55 * hi, (
        f"turn income is direction-asymmetric: CCW {ccw:+.1f} vs "
        f"CW {cw:+.1f} — check the yaw sign chain before any turn arm.")


# --------------------------------------------------------------------------
# WALK bank — commanded forward walking under the CHAMPION reward stack
# (WALK_OVERRIDES above). The honest walker is the scripted tripod
# gait (linux_control/tripod_gait.py) — the trajectory the operator
# TAPE-MEASURED on hardware 2026-08-10: it slips ~50% of its stride on
# concrete and still travels. If the reward stack pays a stall or a
# park more than the one gait proven to walk on the real robot, the
# stack is optimizing against reality.


def _make_walk_env(seed: int, overrides: dict | None = None,
                   episode_seconds: float = 15.0):
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv

    cfg = load_config()
    for (sec, leaf), val in (overrides or WALK_OVERRIDES).items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=False,
        dr_scale=0.0, episode_seconds=episode_seconds, seed=seed, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad", "walk"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "walk" else 0.0)
    return env


def _walk_rollout(policy: str, seed: int, *, vx: float = WALK_CMD_VX,
                  vy: float = 0.0,
                  overrides: dict | None = None,
                  stance: tuple[float, float] = WALK_PLANT) -> float:
    from tripod_gait import TripodGait

    env = _make_walk_env(seed, overrides)
    env.reset()
    # Pin the command deterministically: hold 1 s, ramp 1 s, then a
    # constant (vx, vy) command (overwrites the sampled trajectory
    # arrays in place — same shapes, same goal machinery).
    traj = env._goal_traj
    n = len(traj.vx)
    hold_n = ramp_n = int(round(1.0 / env.dt))
    ramp = np.linspace(0.0, 1.0, ramp_n)
    traj.vx[:] = vx
    traj.vx[:hold_n] = 0.0
    traj.vx[hold_n:hold_n + ramp_n] = vx * ramp
    traj.vy[:] = vy
    traj.vy[:hold_n] = 0.0
    traj.vy[hold_n:hold_n + ramp_n] = vy * ramp
    if traj.wz is not None:
        traj.wz[:] = 0.0

    # "skate" = the same tripod gait with ZERO swing lift: feet never
    # leave the ground, strides happen by sliding — the scripted twin
    # of the learned paddle/drag walkers (slip/m ~1, audit 08-11).
    gait = TripodGait(vx=0.0, lift=0.0 if policy == "skate" else 0.025)
    gait.sync_plant_stance(*stance)
    plant_rad = np.array([0.0, *stance] * 6) * DEG2RAD
    gait.reset_phase()

    total, step = 0.0, 0
    while True:
        t = step * env.dt
        i = min(step, n - 1)
        if policy in ("gait", "skate"):
            gait.set_velocity(vx=float(traj.vx[i]), vy=float(traj.vy[i]))
            act = q_rad_to_action(np.asarray(gait.desired_deg(t)) * DEG2RAD)
        elif policy == "stall":       # march in place: steps, no stride
            gait.set_velocity(vx=0.0, vy=0.0)
            act = q_rad_to_action(np.asarray(gait.desired_deg(t)) * DEG2RAD)
        else:                         # park: hold the plant, refuse
            act = q_rad_to_action(plant_rad)
        _obs, r, term, trunc, _info = env.step(act)
        total += float(r)
        step += 1
        if term or trunc:
            break
    env.close()
    return total


@pytest.fixture(scope="module")
def walk_returns() -> dict[str, float]:
    return {p: float(np.mean([_walk_rollout(p, s) for s in SEEDS]))
            for p in ("gait", "stall", "park")}


def test_walk_progress_beats_paddle_stall(walk_returns):
    """The hardware-proven scripted gait (slips ~50% of its stride ON
    CONCRETE, tape 08-10, and still walks) must out-earn a march-in-
    place stall by a wide margin under the champion stack. Physical
    foot slip alone is NOT failure — a stack that charges the only
    gait known to work on the robot below a stall is miscalibrated.
    (First run 08-10: gait +769 / stall +409 / park +217 — passing.)"""
    gait, stall = walk_returns["gait"], walk_returns["stall"]
    assert gait > 1.5 * stall and gait > stall + 100.0, (
        f"march-in-place rivals the hardware-proven gait: "
        f"{walk_returns} — slip is being priced as failure.")


def test_walk_stall_beats_refusal_park(walk_returns):
    """Required ordering (module docstring): stall > park. A stepping
    stall is at least TRYING; parking through a move command is
    refusal and must earn less (park-duty pricing does this today)."""
    assert walk_returns["stall"] > walk_returns["park"], (
        f"refusing to step out-earns trying: {walk_returns}")


# --------------------------------------------------------------------------
# HEIGHT bank — the height-keeping income gate (08-10, hardware
# finding rl_docs/HARDWARE.md "sag": every deployed walk policy
# migrates to a crouch 54-70 mm below the spawn stance; knees track
# commands, so the crouch is COMMANDED posture the reward stack never
# priced — walk income at ~3/tick outbids the base k_height charge of
# ~0.36/tick). The gate multiplies velocity income by a Gaussian on
# body height vs the episode anchor (reward.walk_height_gate,
# reward.walk_height_sigma_mm). The crouch walker below rides ~-51 mm
# — the same depth as the hardware sag — and still makes progress, so
# it is exactly the exploit the gate must devalue WITHOUT taxing the
# upright hardware-proven gait.

HGT_OVERRIDES = dict(WALK_OVERRIDES)
HGT_OVERRIDES.update({
    ("reward", "walk_height_gate"): 1.0,
    ("reward", "walk_height_sigma_mm"): 30.0,
})
WALK_CROUCH = (-20.0, 105.0)   # rides ~-51 mm, walks ~0.03 m/s (probe 08-10)


@pytest.fixture(scope="module")
def height_returns() -> dict[str, float]:
    def mean(stance, ov):
        return float(np.mean([
            _walk_rollout("gait", s, overrides=ov, stance=stance)
            for s in SEEDS]))
    return {
        "upright_gated": mean(WALK_PLANT, HGT_OVERRIDES),
        "crouch_gated": mean(WALK_CROUCH, HGT_OVERRIDES),
        "upright_ungated": mean(WALK_PLANT, WALK_OVERRIDES),
        "crouch_ungated": mean(WALK_CROUCH, WALK_OVERRIDES),
    }


def test_height_gate_pays_upright_over_crouch(height_returns):
    """Under the gated stack the upright gait must out-earn the same
    gait walking 50 mm low by a wide margin — the crouch is the
    measured hardware sag posture and must not remain a paid basin.
    (First run 08-10: upright +542 / crouch +43 single-seed probe.)"""
    up, lo = height_returns["upright_gated"], height_returns["crouch_gated"]
    assert up > 2.0 * lo and up > lo + 150.0, (
        f"crouch-walking rivals upright walking under the height gate: "
        f"{height_returns}")


def test_height_gate_bites_the_crouch(height_returns):
    """The gate itself must do the work: the crouch walker's return
    must drop hard when the gate turns on, otherwise the ordering
    above is just the pre-existing k_height charge and the gate is
    dead weight."""
    on, off = height_returns["crouch_gated"], height_returns["crouch_ungated"]
    assert on < 0.65 * off, (
        f"height gate barely moves the crouch return ({on:.0f} vs "
        f"{off:.0f} ungated) — gate is not engaging.")


def test_height_gate_light_tax_on_honest_gait(height_returns):
    """The upright hardware-proven gait bobs a few mm; the gate must
    charge it almost nothing (factor ~0.99 measured). A gate that
    taxes the honest gait is miscalibrated (sigma too tight)."""
    on, off = (height_returns["upright_gated"],
               height_returns["upright_ungated"])
    assert on > 0.90 * off, (
        f"height gate taxes the upright gait ({on:.0f} vs {off:.0f} "
        f"ungated) — walk_height_sigma_mm too tight.")


# --------------------------------------------------------------------------
# OMNI bank — the cw-omni-mirror lineage stack (08-10): full-circle
# velocity commands + the yaw command set + the deployment contract
# (meas := ref, 25 deg tilt) + k_current=0 (hardware ruling: walking
# measured CHEAPER than standing; sim current pricing untrusted).
# MDP_PREFLIGHT for any arm that trains on "walk where the joystick
# points": the hardware-proven gait must out-earn stall and park in
# EVERY commanded direction, not just forward, and the turn ordering
# must survive the re-priced stack. Start-pose diversity keys
# (walk_park_start_frac, walk_turn_in_place_frac) and command-sampling
# keys are deliberately NOT set here — the bank pins its own commands
# and start pose; those keys do not touch per-tick pricing.

OMNI_OVERRIDES = dict(TURN_OVERRIDES)
OMNI_OVERRIDES.update({
    ("reward", "k_current"): 0.0,
    ("goal", "walk_obs_body_vel"): 2.0,
    ("safety", "max_roll_deg"): 25.0,
    ("safety", "max_pitch_deg"): 25.0,
    # Achieved-yaw kernel gate (08-11): closes the turn-in-place freeze
    # floor that collapsed cw-omni-mirror1-r1 — see the freeze-floor
    # bank at the bottom of this file. Any omni arm MUST train with it.
    ("reward", "walk_kernel_yaw_gate"): 1.0,
})

# name -> (vx, vy) at the champion speed band; diag keeps |v| in band
OMNI_CMDS = {
    "forward": (WALK_CMD_VX, 0.0),
    "backward": (-WALK_CMD_VX, 0.0),
    "crab_left": (0.0, WALK_CMD_VX),
    "diag_back_right": (-WALK_CMD_VX * 0.707, -WALK_CMD_VX * 0.707),
}


@pytest.fixture(scope="module")
def omni_returns() -> dict[str, dict[str, float]]:
    return {name: {p: float(np.mean(
        [_walk_rollout(p, s, vx=vx, vy=vy, overrides=OMNI_OVERRIDES)
         for s in SEEDS]))
        for p in ("gait", "stall", "park")}
        for name, (vx, vy) in OMNI_CMDS.items()}


def test_omni_gait_beats_stall_and_park_in_every_direction(omni_returns):
    """A full-circle arm is only launchable if honest locomotion
    out-earns march-in-place AND refusal for every command direction —
    otherwise PPO will find the direction where stalling pays and park
    the rest (the walk park attractor, directionally)."""
    for name, r in omni_returns.items():
        assert r["gait"] > r["stall"] + 50.0, (
            f"{name}: stall rivals the gait: {r}")
        assert r["gait"] > r["park"] + 50.0, (
            f"{name}: parking rivals the gait: {r}")


def test_omni_directions_priced_comparably(omni_returns):
    """No commanded direction may be structurally cheap or expensive:
    the gait's income for the worst direction must stay within 45% of
    the best (matches the CW/CCW turn-symmetry tolerance). A skewed
    stack re-trains the drift/heading bias back in."""
    gains = {n: r["gait"] for n, r in omni_returns.items()}
    lo, hi = min(gains.values()), max(gains.values())
    assert lo > 0.55 * hi, (
        f"direction income skew: {gains} — full-circle commands are "
        "not priced evenly; fix before training an omni arm.")


def test_trans_only_gait_beats_stall_and_park_every_direction():
    """TRANSLATION-ONLY stack (cw-omni-trans1, operator 08-10 late):
    the dep1 recipe + full-circle headings + k_current=0, NO yaw stack
    at all — turning de-scoped (no camera = no meaningful front; every
    freeze-exploit ingredient lived in the turn machinery). Ordering
    must hold in all four directions without k_yaw_still/turn terms in
    the stack, since removing a charge changes every tick's total."""
    trans = {k: v for k, v in OMNI_OVERRIDES.items()
             if k[1] not in ("walk_yaw_cmd", "k_walk_yaw",
                             "walk_yaw_kernel_gate", "k_yaw_prog",
                             "k_yaw_still", "walk_yaw_max_rad_s",
                             "walk_yaw_zero_frac",
                             "walk_turn_in_place_frac")}
    for name, (vx, vy) in OMNI_CMDS.items():
        r = {p: float(np.mean(
            [_walk_rollout(p, s, vx=vx, vy=vy, overrides=trans)
             for s in SEEDS]))
            for p in ("gait", "stall", "park")}
        assert r["gait"] > r["stall"] + 50.0, f"{name}: {r}"
        assert r["gait"] > r["park"] + 50.0, f"{name}: {r}"


def test_drag_stance_stack_prices_skating_below_stepping():
    """STRUCTURAL DRAG-CHARGE stack (charge-magnitude audit 08-11,
    probe_drag_audit.py): translation-only stack + k_drag_stance at the
    audit-derived operating point (k=8000/m over a 6 mm per-stance
    allowance, 0.25 mm/tick jitter floor). The launch gate for the
    from-scratch arm: honest STEPPING must out-earn the zero-lift
    SKATE of the same gait (the exact behavior every learned walker
    converged to — per-tick k_drag_loaded could never create this
    ordering at any coefficient), while the old orderings (gait >
    stall > park) survive the new charge."""
    stack = {k: v for k, v in OMNI_OVERRIDES.items()
             if k[1] not in ("walk_yaw_cmd", "k_walk_yaw",
                             "walk_yaw_kernel_gate", "k_yaw_prog",
                             "k_yaw_still", "walk_yaw_max_rad_s",
                             "walk_yaw_zero_frac",
                             "walk_turn_in_place_frac")}
    stack.update({
        ("reward", "k_drag_stance"): 8000.0,
        ("reward", "drag_stance_allow_mm"): 6.0,
        ("reward", "drag_stance_tick_floor_mm"): 0.25,
    })
    for name, (vx, vy) in (("forward", OMNI_CMDS["forward"]),
                           ("crab_left", OMNI_CMDS["crab_left"])):
        r = {p: float(np.mean(
            [_walk_rollout(p, s, vx=vx, vy=vy, overrides=stack)
             for s in SEEDS]))
            for p in ("gait", "skate", "stall", "park")}
        assert r["gait"] > r["skate"] + 50.0, (
            f"{name}: skating rivals stepping under the charge: {r}")
        assert r["gait"] > r["stall"] + 50.0, f"{name}: {r}"
        assert r["gait"] > r["park"] + 50.0, f"{name}: {r}"


def test_omni_turn_ordering_survives_repricing():
    """The TURN bank ordering (turn > partial > drift > park) must hold
    under the omni stack too (k_current=0 + dep contract could in
    principle flip a margin)."""
    t = {p: float(np.mean([_turn_rollout(p, s_wz * TURN_CMD_WZ, s,
                                         overrides=OMNI_OVERRIDES)
                           for s in SEEDS for s_wz in (+1.0, -1.0)]))
         for p in ("turn", "partial", "drift", "park")}
    assert t["turn"] > t["partial"] > t["drift"] > t["park"], t


# --------------------------------------------------------------------------
# OMNI freeze-floor bank — the cw-omni-mirror1-r1 park/freeze exploit
# (08-11). The 40M hardening run collapsed to standing still: frozen
# episodes scored ~1130 while its real walking episodes scored 500-860.
# Root cause (probe-confirmed, NOT the k_yaw_still guess in TURN.md):
# on turn-in-place ticks (s_ref ~ 0, wz_ref != 0) the LINEAR velocity
# kernel pays a frozen robot FULL income (v_lin = 0 = ref) because
# walk_kernel_prog_gate only engages when s_ref > 1e-3 — a freeze
# banked 750/ep of blind kernel income plus ~357/ep task income, an
# income floor no mid-training walker could beat. The same hole left
# k_park_duty / k_step_event inert (they also require s_ref > 1e-3).
# Fix under test: reward.walk_kernel_yaw_gate — on yaw-commanded
# zero-linear ticks, multiply the linear kernel by achieved-yaw
# fraction clip(wz/wz_ref, 0, 1), the exact analog of the prog gate.
# These tests FAILED against the pre-fix stack (freeze at 0.78x full
# turn) and must keep failing any future stack that re-opens the hole.

# The r1 training episode mixture (resample/stops/turn-in-place) — the
# distribution the exploit was FOUND in; constant-command tests alone
# let it through.
OMNI_MIX_OVERRIDES = dict(OMNI_OVERRIDES)
OMNI_MIX_OVERRIDES.update({
    ("goal", "walk_heading_max_rad"): math.pi,
    ("goal", "walk_cmd_resample_s"): 1.5,
    ("goal", "walk_cmd_resample_jitter"): 0.6,
    ("goal", "walk_cmd_blend_s_min"): 0.1,
    ("goal", "walk_cmd_blend_s_max"): 1.0,
    ("goal", "walk_stop_frac"): 0.2,
    ("goal", "walk_turn_in_place_frac"): 0.30,
})
MIX_SEEDS = (0, 1, 2, 3, 4, 5)   # 0,1,2,5 draw turn-in-place episodes,
                                 # 3,4 draw mixed linear episodes


def _mix_rollout(policy: str, seed: int) -> float:
    """Roll the SAMPLED (training-distribution) command trajectory:
    scripted gait following every command vs a frozen plant hold."""
    from tripod_gait import TripodGait

    env = _make_walk_env(seed, OMNI_MIX_OVERRIDES)
    env.reset()                      # keep the sampled mixture
    traj = env._goal_traj
    n = len(traj.vx)
    gait = TripodGait(vx=0.0)
    gait.sync_plant_stance(*WALK_PLANT)
    plant_rad = np.array([0.0, *WALK_PLANT] * 6) * DEG2RAD
    gait.reset_phase()
    total, step = 0.0, 0
    while True:
        t = step * env.dt
        i = min(step, n - 1)
        if policy == "gait":
            wz = float(traj.wz[i]) if traj.wz is not None else 0.0
            gait.set_velocity(vx=float(traj.vx[i]),
                              vy=float(traj.vy[i]), omega=wz)
            act = q_rad_to_action(np.asarray(gait.desired_deg(t)) * DEG2RAD)
        else:                        # freeze: hold the plant, ignore all
            act = q_rad_to_action(plant_rad)
        _obs, r, term, trunc, _info = env.step(act)
        total += float(r)
        step += 1
        if term or trunc:
            break
    env.close()
    return total


def test_omni_freeze_floor_on_turn_in_place():
    """A freeze during a commanded turn-in-place must earn a SMALL
    fraction of honest turning — losing by a wide margin, not tying
    (TURN.md, 08-11). Pre-fix this failed at park = 0.78x turn: the
    blind linear kernel alone guaranteed parking most of a perfect
    turner's income, so PPO parked the moment real turning got hard."""
    t = {p: float(np.mean([_turn_rollout(p, s_wz * TURN_CMD_WZ, s,
                                         overrides=OMNI_OVERRIDES)
                           for s in SEEDS for s_wz in (+1.0, -1.0)]))
         for p in ("turn", "partial", "park")}
    assert t["park"] < 0.5 * t["turn"], (
        f"freeze/park collects {t['park']/t['turn']:.2f} of full-turn "
        f"income — the cw-omni-mirror1-r1 park exploit is open: {t}")
    assert t["park"] < t["partial"] - 100.0, (
        f"parking rivals an honest partial (35%) turner: {t}")


def test_omni_episode_mixture_gait_beats_freeze():
    """Across SAMPLED training-distribution episodes (turn-in-place,
    stops, heading resamples — the exact mixture cw-omni-mirror1-r1
    trained on), following the commands must beat a full-episode freeze
    on EVERY draw, and the freeze's mean income floor must stay well
    below half of honest income. Constant-command tests passed while
    the exploit was open — this mixture-level check is the tripwire."""
    gait = {s: _mix_rollout("gait", s) for s in MIX_SEEDS}
    freeze = {s: _mix_rollout("freeze", s) for s in MIX_SEEDS}
    for s in MIX_SEEDS:
        assert gait[s] > freeze[s] + 100.0, (
            f"seed {s}: freeze ({freeze[s]:+.0f}) rivals the gait "
            f"({gait[s]:+.0f}) on a training-distribution episode")
    g_mean = float(np.mean(list(gait.values())))
    f_mean = float(np.mean(list(freeze.values())))
    assert f_mean < 0.45 * g_mean, (
        f"freeze income floor {f_mean:+.0f} is {f_mean/g_mean:.2f}x of "
        f"honest {g_mean:+.0f} — parking is still a paid basin.")


# --------------------------------------------------------------------------
# TURN-STACK stillness-subsidy bank — the probe-found latent defect
# (08-11, probe_walk_income on the mirror2 stack). On LINEAR-command
# ticks the ungated heading-hold side of the yaw kernel (k_walk_yaw at
# wz_ref = 0) paid a MOTIONLESS body full income — 373-375/ep to
# freeze, sacrifice and paddle alike, the single largest channel in
# the stack — while k_yaw_still charged the honest gait's natural
# zero-mean wz oscillation ~-73/ep and the degenerates ~0. Net: the
# yaw stack taxed honest straight walking ~-100/ep RELATIVE to body
# stillness. This is exactly the hole the old TURN bank missed: it
# checked the new mechanism terms in isolation on turn-in-place
# commands, never the FULL summed stack on a straight-line command
# against a body that simply doesn't move (the walk-freeze bug hid in
# the same blind spot). Fixes under test (walk_task.py, both
# cfg-gated, default 0 = legacy):
#   reward.walk_yaw_hold_prog_gate — heading-hold yaw income times
#       achieved-linear-progress fraction clip(along/s_ref, 0, 1);
#       genuine stop segments (s_ref ~ 0) stay paid.
#   reward.yaw_still_avg_s — the drift charge prices the EMA of wz
#       (the DC drift) instead of the instantaneous wz (the honest
#       gait's zero-mean stride oscillation).
# The legacy assertions below prove the subsidy EXISTED on the stack
# mirror2 actually trained with — if walk_task's defaults ever change,
# they scream.

# Scripted omega calibrated so the ACHIEVED body rotation matches the
# measured structural drift (the fingerprint being priced): while
# translating, the scripted gait only realizes ~40% of its commanded
# omega (slip), so commanding 0.09 achieves ~0.04 — a rider that
# rotates half the real drift is the wrong reference. Measured
# (seed 0, this stack): cmd 0.25 -> achieved wz mean +0.088 rad/s.
DRIFT_RIDE_WZ = 0.25


def _walk_rollout_terms(policy: str, seed: int,
                        overrides: dict) -> tuple[float, dict]:
    """_walk_rollout with per-term accounting (info['reward_*'] sums)
    on a pinned straight-line command; adds the 'driftride' policy —
    walk the command but let the body rotate at the structural drift
    (the cheat that collects heading-hold yaw income for free)."""
    from tripod_gait import TripodGait

    env = _make_walk_env(seed, overrides)
    env.reset()
    traj = env._goal_traj
    n = len(traj.vx)
    hold_n = ramp_n = int(round(1.0 / env.dt))
    ramp = np.linspace(0.0, 1.0, ramp_n)
    traj.vx[:] = WALK_CMD_VX
    traj.vx[:hold_n] = 0.0
    traj.vx[hold_n:hold_n + ramp_n] = WALK_CMD_VX * ramp
    traj.vy[:] = 0.0
    if traj.wz is not None:
        traj.wz[:] = 0.0

    gait = TripodGait(vx=0.0)
    gait.sync_plant_stance(*WALK_PLANT)
    plant_rad = np.array([0.0, *WALK_PLANT] * 6) * DEG2RAD
    gait.reset_phase()

    total, step = 0.0, 0
    terms: dict[str, float] = {}
    while True:
        t = step * env.dt
        i = min(step, n - 1)
        if policy == "gait":
            gait.set_velocity(vx=float(traj.vx[i]), vy=0.0)
            act = q_rad_to_action(np.asarray(gait.desired_deg(t)) * DEG2RAD)
        elif policy == "driftride":
            gait.set_velocity(vx=float(traj.vx[i]), vy=0.0,
                              omega=DRIFT_RIDE_WZ if step >= hold_n
                              else 0.0)
            act = q_rad_to_action(np.asarray(gait.desired_deg(t)) * DEG2RAD)
        else:                         # freeze: hold the plant, refuse
            act = q_rad_to_action(plant_rad)
        _obs, r, term, trunc, info = env.step(act)
        total += float(r)
        for k, v in info.items():
            if k.startswith("reward_"):
                terms[k] = terms.get(k, 0.0) + float(v)
        step += 1
        if term or trunc:
            break
    env.close()
    return total, terms


def _yaw_net(terms: dict) -> float:
    return (terms.get("reward_walk_yaw", 0.0)
            + terms.get("reward_yaw_still", 0.0))


@pytest.fixture(scope="module")
def subsidy_returns() -> dict[str, dict[str, list]]:
    out: dict[str, dict[str, list]] = {}
    for stack, ov in (("fixed", TURN_OVERRIDES),
                      ("legacy", TURN_LEGACY_OVERRIDES)):
        out[stack] = {p: [_walk_rollout_terms(p, s, ov) for s in SEEDS]
                      for p in ("gait", "driftride", "freeze")}
    return out


def test_yaw_stack_stillness_subsidy_is_closed(subsidy_returns):
    """Under the FIXED turn stack, a frozen body on a straight-line
    command must collect (nearly) none of the heading-hold yaw income,
    and the honest gait's NET yaw channel (kernel + drift charge) must
    beat the freeze's by a wide margin. Pre-fix this was upside down:
    freeze +375/ep vs gait ~+230/ep net."""
    gait = float(np.mean([_yaw_net(t) for _, t in
                          subsidy_returns["fixed"]["gait"]]))
    frz = float(np.mean([_yaw_net(t) for _, t in
                         subsidy_returns["fixed"]["freeze"]]))
    assert gait > frz + 100.0, (
        f"yaw channel still subsidizes stillness on straight-line "
        f"commands: gait net {gait:+.0f} vs freeze net {frz:+.0f}")


def test_yaw_stack_subsidy_existed_on_legacy_stack(subsidy_returns):
    """The defect this bank pins must be REAL on the stack the mirror2
    lineage actually trained with: the frozen body's net yaw income
    rivals/beats the honest gait's there. If this ever fails, the
    legacy reproduction drifted and the bank is testing air."""
    gait = float(np.mean([_yaw_net(t) for _, t in
                          subsidy_returns["legacy"]["gait"]]))
    frz = float(np.mean([_yaw_net(t) for _, t in
                         subsidy_returns["legacy"]["freeze"]]))
    assert frz > gait - 50.0, (
        f"legacy stack no longer shows the stillness subsidy (gait "
        f"{gait:+.0f} vs freeze {frz:+.0f}) — walk_task defaults "
        f"changed under this bank; re-derive the legacy overrides.")


def test_yaw_stack_no_net_tax_on_honest_straight_walk(subsidy_returns):
    """FULL summed stack: turning the (fixed) yaw stack on must not
    erode honest straight walking's margin over a freeze. Pre-fix the
    yaw terms shrank that margin by ~475/ep (subsidy to the freeze +
    oscillation tax on the gait) — the exact economics that let
    mirror1-r1 park."""
    def margin(stack: str) -> float:
        g = float(np.mean([tot for tot, _ in
                           subsidy_returns[stack]["gait"]]))
        f = float(np.mean([tot for tot, _ in
                           subsidy_returns[stack]["freeze"]]))
        return g - f
    assert margin("fixed") > margin("legacy") + 200.0, (
        f"the fixes do not restore honest walking's margin over a "
        f"freeze: fixed {margin('fixed'):+.0f} vs legacy "
        f"{margin('legacy'):+.0f}")
    assert margin("fixed") > 300.0, (
        f"honest straight walking barely beats a freeze under the "
        f"fixed turn stack: margin {margin('fixed'):+.0f}")


def test_drift_rider_never_beats_honest_straight_walk(subsidy_returns):
    """The drift-rider cheat — track the linear command but let the
    body rotate at the structural +0.09 rad/s drift, collecting the
    yaw kernel's heading-hold band — must lose to the honest straight
    gait under the FULL summed fixed stack. This is the summed-stack
    check the old TURN bank never ran (it priced turn-in-place
    commands only)."""
    gait = float(np.mean([tot for tot, _ in
                          subsidy_returns["fixed"]["gait"]]))
    ride = float(np.mean([tot for tot, _ in
                          subsidy_returns["fixed"]["driftride"]]))
    assert gait > ride + 50.0, (
        f"riding the structural drift rivals honest straight walking "
        f"under the fixed stack: gait {gait:+.0f} vs rider {ride:+.0f}")


# --------------------------------------------------------------------------
# HOLD bank — quiet stand at the plant under the stand-line stack
# (cw-stand-bc1/-hard1 lineage cfg + reward.hold_still_gate=1).
#
# The failure this bank pins (cw-stand-bc1-hard1 dig-in, 08-11): hold
# and track episodes were never quiet stands — the tracking kernel pays
# torso attitude/height with NO opinion on the legs, so the policy
# cycles legs continuously (duty 0.85/0.09, 6-19 swings per 15 s
# episode; feet 12-50 mm up at 2M steps, 100-161 mm splayed at 10M)
# while collecting near-full income. k_still is a BONUS (default 0)
# and charges nothing; a sparse 10-frame video strip missed it at two
# separate verdicts. reward.hold_still_gate scales the kernel on
# hold/track ticks by feet-down^2 x hard no-flag (PLANT_SPEC
# flag_leg_mm) x stillness (only while the reference is stationary,
# so TRACK's commanded motion is never charged).
#
# Required ordering: quiet stand > continuous stepping > flag-leg
# park; the gate must do the work (stepping loses hard when it turns
# on) and must NOT tax the honest quiet stand.

from rl_move.sim.sim_env import PLANT_SPEC  # noqa: E402

PLANT_SPEC_FLAG_MM = float(PLANT_SPEC["flag_leg_mm"])
HOLD_OVERRIDES = dict(SCORE_OVERRIDES)
HOLD_OVERRIDES.update({
    ("reward", "hold_still_gate"): 1.0,
})
HOLD_LEGACY = dict(SCORE_OVERRIDES)   # gate off = today's stand stack


def _make_hold_env(seed: int, overrides=None) -> SimHexapodJointGoalEnv:
    cfg = load_config()
    for (sec, leaf), val in (overrides or HOLD_OVERRIDES).items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointGoalEnv(
        params=SimServoParams.from_cfg(None), randomize=False,
        dr_scale=0.0, episode_seconds=15.0, seed=seed, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "hold" else 0.0)
    return env


def _hold_rollout(policy: str, seed: int, overrides=None) -> dict:
    """Scripted hold-mode policies from the plant start.

    quiet     hold the settled plant pose exactly (the intended skill).
    stepping  the 2M-checkpoint pathology: alternating tripods lift
              continuously (~1 Hz) while the torso stays level at
              height — swing feet stay BELOW flag_leg_mm, so only the
              feet-down fraction and the stillness factor charge it.
    flag      the 10M-checkpoint pathology (harness worst-foot
              162-189 mm across hold/lower): one front leg parked high
              in the air, five legs hold the plant — body level at
              height, a FROZEN pose, so stillness alone cannot charge
              it; the hard no-flag zero must. (A scripted BOTH-front-
              legs splay nose-dives and terminates — position-control
              can't rebalance; the single flag is the stable scripted
              form of the same class.)
    """
    env = _make_hold_env(seed, overrides)
    env.reset()
    q0 = env.data.qpos[env._qadr].copy()
    # Calibrated on the plant (probe 08-11): hip -30 deg lifts the
    # foot ~50 mm (honest swing band); hip -50 + knee -50 parks it
    # ~190 mm up (the observed splay class).
    lift = 30.0 * DEG2RAD
    total, step = 0.0, 0
    max_clear = 0.0
    while True:
        if policy == "quiet":
            act = q_rad_to_action(q0)
        elif policy == "stepping":
            q = q0.copy()
            phase = int(step * env.dt / 0.5) % 2   # 0.5 s per tripod
            for leg in ((0, 2, 4) if phase == 0 else (1, 3, 5)):
                q[3 * leg + 1] -= lift
            act = q_rad_to_action(q)
        else:  # flag
            q = q0.copy()
            q[1] -= 50.0 * DEG2RAD          # leg 0 hip up
            q[2] -= 50.0 * DEG2RAD          # leg 0 knee curled
            act = q_rad_to_action(q)
        _obs, r, term, trunc, _info = env.step(act)
        total += float(r)
        step += 1
        clear = max(float(env.data.xpos[b, 2]) - env._pad_z_ref[i]
                    for i, b in enumerate(env._pad_bids))
        max_clear = max(max_clear, clear)
        if term or trunc:
            terminated = term
            break
    h_err = (float(env.data.xpos[env._chassis_bid, 2]) - env._z0)
    plant_ok, detail = ((False, {"terminated": True}) if terminated
                        else env.plant_report(height_err_m=h_err))
    env.close()
    return {"ret": total, "plant": bool(plant_ok),
            "max_clear_mm": max_clear * 1000.0, "detail": detail}


HOLD_POLICIES = ("quiet", "stepping", "flag")


@pytest.fixture(scope="module")
def hold_bank() -> dict[str, list[dict]]:
    return {p: [_hold_rollout(p, s) for s in SEEDS]
            for p in HOLD_POLICIES}


@pytest.fixture(scope="module")
def hold_returns(hold_bank) -> dict[str, float]:
    return {p: float(np.mean([r["ret"] for r in rolls]))
            for p, rolls in hold_bank.items()}


@pytest.fixture(scope="module")
def hold_legacy_returns() -> dict[str, float]:
    return {p: float(np.mean([_hold_rollout(p, s, HOLD_LEGACY)["ret"]
                              for s in SEEDS]))
            for p in ("quiet", "stepping")}


def test_hold_bank_policies_are_the_right_shapes(hold_bank):
    """Self-check that the scripted policies reproduce the two observed
    pathologies: the stepping swing feet must stay BELOW the flag-leg
    threshold (the 2M pathology was 12-50 mm; if a bank swing crosses
    60 mm the hard zero fires and the test stops discriminating
    stillness), and the flag must park a foot well ABOVE it."""
    for r in hold_bank["stepping"]:
        assert 20.0 < r["max_clear_mm"] < PLANT_SPEC_FLAG_MM, (
            f"stepping swing peak {r['max_clear_mm']:.0f}mm is outside "
            f"the honest-magnitude band (20-{PLANT_SPEC_FLAG_MM:.0f}mm)")
    for r in hold_bank["flag"]:
        assert r["max_clear_mm"] > PLANT_SPEC_FLAG_MM + 20.0, (
            f"flag foot only reaches {r['max_clear_mm']:.0f}mm — not "
            f"the observed 100-190mm flag pose")


def test_hold_quiet_dominates_continuous_stepping(hold_returns):
    """Under the gated stack the quiet stand must out-earn continuous
    leg-cycling decisively — the pathology PPO actually converged to
    must not remain a paid basin."""
    q, s = hold_returns["quiet"], hold_returns["stepping"]
    assert q > 1.5 * s and q > s + 100.0, (
        f"continuous stepping rivals the quiet stand under the gate: "
        f"{hold_returns}")


def test_hold_stepping_beats_flag_leg(hold_returns):
    """Both are failures, but the persistent flag leg (one foot parked
    ~190 mm up, hard no-flag zero) must earn strictly less than
    honest-magnitude stepping — the gradient must point from the flag
    pose back toward feet-down behavior, not sideways."""
    s, f = hold_returns["stepping"], hold_returns["flag"]
    assert s > f + 50.0, (
        f"the frozen flag-leg pose rivals feet-down stepping: "
        f"{hold_returns}")


def test_hold_gate_bites_the_stepping(hold_returns, hold_legacy_returns):
    """The gate itself must do the work: stepping's return must drop
    hard when the gate turns on. Pre-fix (legacy stack) continuous
    stepping collected ~parity with the quiet stand — that is the
    bank's reason to exist."""
    on, off = hold_returns["stepping"], hold_legacy_returns["stepping"]
    assert on < 0.65 * off, (
        f"hold_still_gate barely moves the stepping return ({on:.0f} "
        f"vs {off:.0f} ungated) — gate is not engaging.")


def test_hold_gate_light_tax_on_quiet_stand(hold_returns,
                                            hold_legacy_returns):
    """A real quiet stand (small servo jitter only) must keep ~full
    pay under the gate; taxing honesty is how gates get gamed."""
    on, off = hold_returns["quiet"], hold_legacy_returns["quiet"]
    assert on > 0.90 * off, (
        f"hold_still_gate taxes the honest quiet stand ({on:.0f} vs "
        f"{off:.0f} ungated) — factor miscalibrated.")


def test_hold_quiet_ends_valid_plant(hold_bank):
    """End-state PLANT_SPEC: the quiet stand must end a valid plant on
    every seed; the flag pose must fail it by construction."""
    for r in hold_bank["quiet"]:
        assert r["plant"], (
            f"the quiet stand fails PLANT_SPEC: {r['detail']}")
    for r in hold_bank["flag"]:
        assert not r["plant"], (
            f"the flag-leg pose passes PLANT_SPEC: {r['detail']}")


# --------------------------------------------------------------------------
# HOLD bank, FLAG-FADE variant (reward.hold_flag_fade=1 — from the
# cw-stand-holdstill1 FAIL, 08-11). The hard no-flag zero priced the
# flag park correctly (income 0) but is a ZERO-GRADIENT PLATEAU: the
# trained run kept its leg parked at ~110 mm for all 2M steps because
# every nearby behavior also earned ~0 — nothing said WHICH WAY to
# move. The fade pays a linear ramp over [flag_leg_mm, 2*flag_leg_mm]
# (60->120 mm): compliant poses keep exactly full factor, the observed
# ~110-120 mm park earns scraps WITH a downhill slope toward feet-down,
# and the ~190 mm class still earns 0. These tests pin that the fade
# (a) does not disturb the required quiet > stepping > flag ordering,
# (b) actually creates the gradient (lower park must out-earn higher
# park), and (c) never turns the park into a paid basin (scraps, not
# a living).

HOLD_FADE_OVERRIDES = dict(HOLD_OVERRIDES)
HOLD_FADE_OVERRIDES.update({
    ("reward", "hold_flag_fade"): 1.0,
})


def _hold_fade_rollout(policy: str, seed: int) -> dict:
    """Extra scripted policy 'flag_low': one front leg parked ~113 mm
    up (hip -55, probe-calibrated 08-11) — the exact holdstill1 end
    state class (107-116 mm), high in the fade band — vs the bank's
    'flag' at ~190 mm. Poses LOWER in the band legitimately earn more
    (monotone slope toward the quiet stand is the fade's purpose);
    the binding orderings are against THIS observed class."""
    if policy != "flag_low":
        return _hold_rollout(policy, seed, HOLD_FADE_OVERRIDES)
    env = _make_hold_env(seed, HOLD_FADE_OVERRIDES)
    env.reset()
    q0 = env.data.qpos[env._qadr].copy()
    q = q0.copy()
    q[1] -= 55.0 * DEG2RAD          # leg 0 hip up, knee kept — ~113 mm
    act = q_rad_to_action(q)
    total = 0.0
    max_clear = 0.0
    while True:
        _obs, r, term, trunc, _info = env.step(act)
        total += float(r)
        clear = max(float(env.data.xpos[b, 2]) - env._pad_z_ref[i]
                    for i, b in enumerate(env._pad_bids))
        max_clear = max(max_clear, clear)
        if term or trunc:
            break
    env.close()
    return {"ret": total, "plant": False, "max_clear_mm": max_clear * 1000.0}


@pytest.fixture(scope="module")
def hold_fade_returns() -> dict[str, float]:
    return {p: float(np.mean([_hold_fade_rollout(p, s)["ret"]
                              for s in SEEDS]))
            for p in ("quiet", "stepping", "flag", "flag_low")}


def test_hold_fade_keeps_the_ordering(hold_fade_returns):
    """With the fade on, the binding HOLD ordering must survive:
    quiet > stepping > every flag park."""
    t = hold_fade_returns
    assert t["quiet"] > 1.5 * t["stepping"] and \
        t["quiet"] > t["stepping"] + 100.0, (
        f"fade broke quiet > stepping: {t}")
    assert t["stepping"] > t["flag"] + 50.0 and \
        t["stepping"] > t["flag_low"] + 50.0, (
        f"fade broke stepping > flag park: {t}")


def test_hold_fade_restores_the_gradient(hold_fade_returns):
    """The fade's whole purpose: a lower park must out-earn a higher
    park, so lowering the parked leg is ALWAYS downhill-in-reward.
    Under the hard zero both parks tied at ~0 — the measured
    holdstill1 plateau."""
    lo, hi = hold_fade_returns["flag_low"], hold_fade_returns["flag"]
    assert lo > hi + 20.0, (
        f"no slope: parking at ~120mm ({lo:.0f}) does not out-earn "
        f"parking at ~190mm ({hi:.0f}) — the plateau is still flat")


def test_hold_fade_park_is_scraps_not_a_living(hold_fade_returns):
    """The faded park income must stay scraps (< 25% of the quiet
    stand): partial pay is a gradient, not a destination."""
    t = hold_fade_returns
    assert t["flag_low"] < 0.25 * t["quiet"], (
        f"the ~120mm park collects {t['flag_low']/t['quiet']:.2f}x of "
        f"the quiet stand — the fade re-opened a paid basin: {t}")


# --------------------------------------------------------------------------
# HOLD bank, FEET-LOAD variant (reward.hold_feet_load=1 — from the
# cw-stand-crouchrise1/2/3 trio, 08-11). All three crouch-dose runs
# converged on the SAME hold cheat the two banks above cannot see: two
# legs hover 1-19 mm up. That is below PLANT_SPEC.foot_down_mm (20 mm),
# so the clearance-based feet count prices them as "down", and far
# below flag_leg_mm (60 mm), so neither the hard no-flag zero nor the
# fade fires — full income for a pose the eval's contact-duty clause
# (touch force > 0.5 N) scores at 0.01-0.04 duty. The load gate prices
# MEASURED touch force instead: each unloaded foot multiplies hold
# income by hold_load_floor (0.5), so the two-leg hover earns 0.25 —
# the fade bank's "scraps, not a living" band — while a genuinely
# loaded six-foot stance keeps exactly 1.0.
#
# These tests pin (a) the hover reproduces the crouchrise signature
# (sub-20mm clearance, near-zero contact duty, stable, un-terminated),
# (b) the pre-fix stack REALLY is blind to it (documents the loophole),
# (c) the load gate prices it to scraps, and (d) the honest quiet
# stand is not taxed.

HOLD_LOAD_OVERRIDES = dict(HOLD_FADE_OVERRIDES)
HOLD_LOAD_OVERRIDES.update({
    ("reward", "hold_feet_load"): 1.0,
})

HOVER_LEGS = (1, 4)         # the pair the crouchrise trio parked
HOVER_LIFT_DEG = 8.0        # probe: hip -30 deg ~ 50 mm, so ~13 mm


def _hold_load_rollout(policy: str, seed: int, overrides) -> dict:
    """'quiet' / 'hover' / 'hover1' with per-foot contact-duty telemetry.

    hover     the crouchrise cheat class: two legs held a few mm off
              the ground — a FROZEN pose (stillness can't charge it)
              with all feet inside foot_down_mm (the clearance count
              can't charge it) and nothing near the flag band (no-flag
              can't charge it). Only measured load can.
    hover1    the anchormix1-r1 endgame: exactly ONE leg unloaded (the
              habit sheds one foot — a five-foot stance is sufficient
              and cheaper; product pricing caps its tax at floor 0.5).
    """
    env = _make_hold_env(seed, overrides)
    env.reset()
    q0 = env.data.qpos[env._qadr].copy()
    q = q0.copy()
    if policy == "hover":
        for leg in HOVER_LEGS:
            q[3 * leg + 1] -= HOVER_LIFT_DEG * DEG2RAD
    elif policy == "hover1":
        q[3 * HOVER_LEGS[0] + 1] -= HOVER_LIFT_DEG * DEG2RAD
    act = q_rad_to_action(q)
    total, steps = 0.0, 0
    contact_steps = np.zeros(6)
    lifted_clear = []
    terminated = False
    while True:
        _obs, r, term, trunc, _info = env.step(act)
        total += float(r)
        steps += 1
        for i in range(6):
            if env._touch_adr[i] >= 0 and \
                    float(env.data.sensordata[env._touch_adr[i]]) > 0.5:
                contact_steps[i] += 1
        lifted_clear.append(max(
            float(env.data.xpos[env._pad_bids[i], 2]) - env._pad_z_ref[i]
            for i in HOVER_LEGS))
        if term or trunc:
            terminated = term
            break
    env.close()
    half = lifted_clear[len(lifted_clear) // 2:]   # settled half only
    return {"ret": total, "terminated": terminated,
            "duty": contact_steps / max(steps, 1),
            "lifted_clear_mm": float(np.mean(half)) * 1000.0}


@pytest.fixture(scope="module")
def hold_load_bank() -> dict[str, dict[str, list[dict]]]:
    return {
        "prefix": {p: [_hold_load_rollout(p, s, HOLD_FADE_OVERRIDES)
                       for s in SEEDS] for p in ("quiet", "hover")},
        "load": {p: [_hold_load_rollout(p, s, HOLD_LOAD_OVERRIDES)
                     for s in SEEDS] for p in ("quiet", "hover")},
    }


def _mean_ret(rolls: list[dict]) -> float:
    return float(np.mean([r["ret"] for r in rolls]))


def test_hold_load_hover_matches_the_crouchrise_signature(hold_load_bank):
    """Self-check: the scripted hover must reproduce the observed cheat
    — lifted feet settle between clear-of-ground and foot_down_mm
    (crouchrise ended 1-2 mm up; anywhere in (2, 18) mm fools the
    clearance count identically), carry near-zero contact duty
    (crouchrise read 0.01-0.04), stay stable, and never terminate."""
    for r in hold_load_bank["prefix"]["hover"]:
        assert 2.0 < r["lifted_clear_mm"] < 18.0, (
            f"hover feet settle at {r['lifted_clear_mm']:.1f}mm — "
            f"outside the sub-foot_down_mm cheat band")
        assert max(r["duty"][i] for i in HOVER_LEGS) < 0.2, (
            f"hover legs still carry duty {r['duty']} — not the "
            f"zero-load park the crouchrise trio learned")
        assert not r["terminated"], "the hover pose is not even stable"
    for r in hold_load_bank["prefix"]["quiet"]:
        assert min(r["duty"]) > 0.8, (
            f"the quiet stand itself has an unloaded foot ({r['duty']})"
            f" — the bank can't discriminate load from this start")


def test_hold_load_prefix_stack_is_blind_to_the_hover(hold_load_bank):
    """Documents the loophole: under the FULL pre-fix stack (still gate
    + flag fade, the exact crouchrise config), the two-leg hover must
    collect near-parity with the honest quiet stand. If this ever
    fails, the loophole is closed some other way and the load gate
    needs re-justification."""
    q = _mean_ret(hold_load_bank["prefix"]["quiet"])
    h = _mean_ret(hold_load_bank["prefix"]["hover"])
    assert h > 0.85 * q, (
        f"pre-fix stack already prices the hover ({h:.0f} vs quiet "
        f"{q:.0f}) — the crouchrise cheat should be free here")


def test_hold_load_gate_prices_the_hover_to_scraps(hold_load_bank):
    """The fix: with reward.hold_feet_load=1 the hover must earn
    decisively less than the quiet stand — out of parity and into the
    scraps band (each unloaded foot costs x0.5 on hold income)."""
    q = _mean_ret(hold_load_bank["load"]["quiet"])
    h = _mean_ret(hold_load_bank["load"]["hover"])
    assert h < 0.65 * q and h < q - 100.0, (
        f"load gate barely moves the hover ({h:.0f} vs quiet {q:.0f})")


def test_hold_load_gate_light_tax_on_quiet_stand(hold_load_bank):
    """The honest quiet stand keeps ~full pay: per-foot force at this
    robot's weight is well above hold_load_ref_n, so the product sits
    at 1.0 and only real unloading is charged."""
    on = _mean_ret(hold_load_bank["load"]["quiet"])
    off = _mean_ret(hold_load_bank["prefix"]["quiet"])
    assert on > 0.90 * off, (
        f"hold_feet_load taxes the honest quiet stand ({on:.0f} vs "
        f"{off:.0f} without it) — ref/floor miscalibrated.")


# --------------------------------------------------------------------------
# HOLD bank, MIN-over-feet variant (reward.hold_feet_load_min=1 — the
# pre-registered anchormix1-r1 reopen lever, 08-12). Six straight stand
# runs converged on the SAME endgame: exactly ONE foot parked (duty
# 0.01-0.04) while every supervision/pricing change only moved WHICH
# foot. Under the product gate a single unloaded foot's tax caps at
# hold_load_floor (0.5) — a five-foot stance at half pay is "sufficient
# and cheaper". The min variant makes the WORST foot the whole factor:
# load = max(min_i s_i, hold_load_min_floor 0.1), so the one-foot park
# earns scraps with the same linear on-ramp for slope, and the honest
# six-foot stance keeps exactly 1.0. These tests pin (a) default-off is
# bit-exact vs the product path, (b) the one-foot park drops from the
# product's half-pay band to scraps, (c) the quiet stand is untaxed.

HOLD_LOAD_MIN_OVERRIDES = dict(HOLD_LOAD_OVERRIDES)
HOLD_LOAD_MIN_OVERRIDES.update({
    ("reward", "hold_feet_load_min"): 1.0,
})


@pytest.fixture(scope="module")
def hold_load_min_bank() -> dict[str, dict[str, list[dict]]]:
    return {
        "prod": {p: [_hold_load_rollout(p, s, HOLD_LOAD_OVERRIDES)
                     for s in SEEDS] for p in ("quiet", "hover1")},
        "min": {p: [_hold_load_rollout(p, s, HOLD_LOAD_MIN_OVERRIDES)
                    for s in SEEDS] for p in ("quiet", "hover1")},
    }


def test_hold_load_min_default_off_bit_exact():
    """hold_feet_load_min=0 must reproduce the legacy product path
    EXACTLY (same seed, same rollout) — the switch is default-off and
    bit-exact per the code-first rules."""
    off = dict(HOLD_LOAD_OVERRIDES)
    off[("reward", "hold_feet_load_min")] = 0.0
    a = _hold_load_rollout("hover1", SEEDS[0], HOLD_LOAD_OVERRIDES)
    b = _hold_load_rollout("hover1", SEEDS[0], off)
    assert a["ret"] == b["ret"], (
        f"hold_feet_load_min=0 changed the reward path "
        f"({a['ret']} vs {b['ret']})")


def test_hold_load_min_one_foot_park_reproduces_the_fingerprint(
        hold_load_min_bank):
    """Self-check: the scripted one-leg hover matches the anchormix
    endgame — the parked foot carries near-zero duty, the pose is
    stable and un-terminated."""
    for r in hold_load_min_bank["min"]["hover1"]:
        assert r["duty"][HOVER_LEGS[0]] < 0.2, (
            f"hover1 leg still carries duty "
            f"{r['duty'][HOVER_LEGS[0]]:.2f} — not the observed park")
        assert not r["terminated"], "the hover1 pose is not even stable"


def test_hold_load_min_prices_one_foot_park_to_scraps(hold_load_min_bank):
    """The lever: under the product gate the one-foot park keeps a
    living (its tax caps at floor 0.5); under min pricing it must drop
    decisively — below half the product's take and out of parity with
    the quiet stand."""
    q_min = _mean_ret(hold_load_min_bank["min"]["quiet"])
    h_min = _mean_ret(hold_load_min_bank["min"]["hover1"])
    h_prod = _mean_ret(hold_load_min_bank["prod"]["hover1"])
    # Premise: under the product the park keeps a LIVING (~half pay —
    # measured 0.49x quiet: the 0.5 floor caps the tax and nearly all
    # hold income is the gated kernel), far above the fade bank's
    # scraps band (~0.14x).
    assert h_prod > 0.40 * _mean_ret(hold_load_min_bank["prod"]["quiet"]), (
        f"product pricing already prices the one-foot park to scraps "
        f"({h_prod:.0f}) — the min lever's premise is broken, "
        f"re-justify it")
    assert h_min < 0.75 * h_prod, (
        f"min pricing barely moves the one-foot park "
        f"({h_min:.0f} vs product {h_prod:.0f})")
    assert h_min < 0.5 * q_min and h_min < q_min - 100.0, (
        f"one-foot park still keeps a living under min pricing "
        f"({h_min:.0f} vs quiet {q_min:.0f})")


def test_hold_load_min_keeps_quiet_stand_pay(hold_load_min_bank):
    """A genuinely six-foot-loaded stance keeps ~full pay: min over
    fully-loaded feet is 1.0, identical to the product."""
    q_min = _mean_ret(hold_load_min_bank["min"]["quiet"])
    q_prod = _mean_ret(hold_load_min_bank["prod"]["quiet"])
    assert q_min > 0.95 * q_prod, (
        f"min pricing taxes the honest quiet stand ({q_min:.0f} vs "
        f"product {q_prod:.0f}) — min_floor/ref miscalibrated")


# --------------------------------------------------------------------------
# RISE-ROCK bank (dr.rise_rock_*, 08-11 hardware belly-curl rocking gap).
#
# Bench truth (bench_blast camera sessions, 08-11): the learned rise is
# deterministic-fail on hardware — 5/5 tilt_roll trips at the same tick
# (~9 s mid-curl), rel roll 10.1-10.6°, currents <= 0.27 A — while the
# sim's curl stays <= 1.7° under BOTH actuator fits (probed 08-11: the
# loaded fit does NOT reproduce the rock either, so it is not simple
# lag). The axis models it as a persistent one-side hip/knee fold bias
# on the PHYSICAL servo command of rise-mode episodes: the logical loop
# never sees it, encoders read the true drooped angles, the tilt
# reference stays level. These tests pin (a) default-off is inert,
# (b) the bias genuinely rocks the honest reference curl into the
# hardware's trip band, (c) a counter-command CAN level it (the skill
# is expressible in the action space — the gradient exists), and
# (d) non-rise modes are never perturbed.

ROCK_OVERRIDES = dict(RISE_OVERRIDES)
ROCK_OVERRIDES[("dr", "rise_rock_prob")] = 1.0
ROCK_OVERRIDES[("dr", "rise_rock_deg")] = "10,10"
# NOTE on countering: the STATIC path inverse (q_ref - bias) is NOT the
# closability demonstration — at 10° it clips on ~14% of (path x joint)
# points and on the negative-roll side it fails outright (measured at
# authoring). The learnable skill is FEEDBACK: level the measured roll
# through the same fold pattern, which a dumb P-controller already does
# on both sides (test below).


def _make_rock_env(seed: int, overrides) -> SimHexapodJointGoalEnv:
    cfg = load_config()
    for (sec, leaf), val in overrides.items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointGoalEnv(
        params=SimServoParams.from_cfg(None),
        randomize=(("dr", "rise_rock_prob") in overrides),
        dr_scale=0.0, episode_seconds=16.0, seed=seed, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "rise" else 0.0)
    gen.force_rise_start = "flat"
    return env


def _rock_fold_pattern(roll_deg: float) -> np.ndarray:
    """The tipped-start fold→roll mapping as a joint offset (test-local
    twin of sim_env._rise_rock_offset's math)."""
    fold = abs(roll_deg) / 0.36 * DEG2RAD
    legs = (3, 4, 5) if roll_deg > 0 else (0, 1, 2)
    dq = np.zeros(18)
    for leg in legs:
        dq[3 * leg + 1] -= fold
        dq[3 * leg + 2] += 0.5 * fold
    return dq


def _rock_rollout(seed: int, overrides, counter: bool = False) -> dict:
    """Replay the honest reference curl; optionally run a P-feedback
    leveler on measured roll (gain 0.8, the closability floor). Returns
    peak |rel tilt| over the episode."""
    env = _make_rock_env(seed, overrides)
    env.reset()
    ref = np.load(ROOT / RISE_REF)
    q_ref, ramp_ref = ref["q_rad"], int(ref["ramp_i0"])
    r0, p0 = env._true_roll_pitch()
    peak, step = 0.0, 0
    while True:
        j = ramp_ref + (step - env._rise_ramp_i0)
        q = q_ref[min(max(j, 0), len(q_ref) - 1)].copy()
        if counter:
            r, _p = env._true_roll_pitch()
            err = (r - r0) / DEG2RAD
            if abs(err) > 0.5:
                q = q + _rock_fold_pattern(-err * 0.8)
        _obs, _r, term, trunc, _info = env.step(q_rad_to_action(q))
        step += 1
        r, p = env._true_roll_pitch()
        peak = max(peak, abs(r - r0), abs(p - p0))
        if term or trunc:
            break
    return {"peak_tilt_deg": peak / DEG2RAD, "terminated": bool(term),
            "rock": env._rise_rock_offset()}


def test_rise_rock_default_off_is_inert():
    """No dr.rise_rock override -> no draw, no command bias, and the
    honest replay stays as flat as it always was."""
    for seed in SEEDS:
        out = _rock_rollout(seed, RISE_OVERRIDES)
        assert out["rock"] is None, "rock offset emitted with axis off"
        assert out["peak_tilt_deg"] < 4.0, (
            f"legacy flat-start replay tilts {out['peak_tilt_deg']:.1f}° "
            f"with the axis OFF — baseline broken, not the axis")


def test_rise_rock_bias_rocks_the_honest_curl():
    """With a 10° draw the un-countered reference curl must roll into
    the hardware's measured trip band (>= 6°) — the sim now VISITS the
    states the bench fails in."""
    peaks = [_rock_rollout(s, ROCK_OVERRIDES)["peak_tilt_deg"]
             for s in SEEDS]
    assert max(peaks) >= 6.0, (
        f"rock bias barely moves the curl (peaks {peaks}) — the "
        f"fold→roll mapping does not reproduce the bench signature")


def test_rise_rock_feedback_levels_it():
    """A dumb P-controller on measured roll keeps every rocked curl
    clear of the 10° trip band on BOTH sides at the full 10° dose
    (4.6-6.7° peaks, zero terminations at authoring): the leveling
    skill is closable through the action space, so it is learnable —
    not physically impossible."""
    for seed in SEEDS:
        fixed = _rock_rollout(seed, ROCK_OVERRIDES, counter=True)
        assert fixed["peak_tilt_deg"] < 8.0 and not fixed["terminated"], (
            f"P-feedback does not level the rock "
            f"({fixed['peak_tilt_deg']:.1f}°, "
            f"terminated={fixed['terminated']}) — axis not closable")


def test_rise_rock_never_touches_other_modes():
    """A hold-mode episode with the axis forced on must never see the
    bias — the perturbation is scoped to rise episodes only."""
    cfg = load_config()
    for (sec, leaf), val in ROCK_OVERRIDES.items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointGoalEnv(
        params=SimServoParams.from_cfg(None), randomize=True,
        dr_scale=0.0, episode_seconds=6.0, seed=0, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "hold" else 0.0)
    env.reset()
    assert env._ep_rand is not None \
        and env._ep_rand.rise_rock_roll_deg != 0.0, \
        "draw did not fire at prob 1 — test is vacuous"
    assert env._rise_rock_offset() is None, \
        "rock bias leaked into a hold-mode episode"


# --------------------------------------------------------------------------
# WALK-KICK bank (dr.walk_kick_*, 08-11 hardware takeoff-transient gap).
#
# Bench truth (bench_report over ALL 18 hardware walks, 08-11): every
# walk crosses 5° roll within 0.6-1.5 s of gait start and peaks
# 13-27°, at sustained roll RATES of 11-46 °/s — and whether that
# transient recovers or capsizes is ~a coin flip for BOTH champions.
# Static leans do NOT close the gap (cw-dep-tip1-takeoff25-r1:
# child==parent at the matched 20-25° dose): the sim already recovers
# static tipped starts, what it never visits is the moving excursion.
# The axis injects a TRANSIENT one-side fold pulse (half-sine, net-zero
# terminal offset) on the physical command over the first ~second of
# walk-mode episodes. These tests pin (a) default-off is inert, (b) the
# pulse genuinely rolls a frozen plant stance through the hardware band
# and DIES OUT (transient, not a bias), and (c) non-walk modes are
# never perturbed.

KICK_OVERRIDES = {
    ("dr", "walk_kick_prob"): 1.0,
    ("dr", "walk_kick_deg"): "14,14",
    ("dr", "walk_kick_s"): "0.8,0.8",
}


def _make_kick_env(seed: int, overrides):
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv
    cfg = load_config()
    for (sec, leaf), val in overrides.items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None),
        randomize=(("dr", "walk_kick_prob") in overrides),
        dr_scale=0.0, episode_seconds=6.0, seed=seed, cfg=cfg)
    gen = env._goal_gen
    gen.p_walk = 1.0
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 0.0)
    return env


def _kick_rollout(seed: int, overrides) -> dict:
    """Freeze at the episode's start command (a parked plant) and let
    the kick pulse — if drawn — do its thing. Returns the peak |rel
    roll| plus the peak AFTER the pulse window (to prove it dies out)."""
    env = _make_kick_env(seed, overrides)
    env.reset()
    q0 = env._cmd.copy()
    r0, _ = env._true_roll_pitch()
    dur = (env._ep_rand.walk_kick_dur_s if env._ep_rand is not None
           else 0.0)
    peak = peak_after = 0.0
    saw_offset = False
    for _ in range(int(3.0 / env.dt)):
        if env._walk_kick_offset() is not None:
            saw_offset = True
        _obs, _r, term, trunc, _info = env.step(q_rad_to_action(q0))
        r, _p = env._true_roll_pitch()
        rel = abs(r - r0) / DEG2RAD
        peak = max(peak, rel)
        if env._step_i * env.dt >= dur + 1.0:
            peak_after = max(peak_after, rel)
        if term or trunc:
            break
    return {"peak_deg": peak, "peak_after_deg": peak_after,
            "saw_offset": saw_offset, "terminated": bool(term)}


def test_walk_kick_default_off_is_inert():
    """No dr.walk_kick override -> no draw, no command pulse, and the
    frozen plant stays level."""
    for seed in SEEDS:
        out = _kick_rollout(seed, {})
        assert not out["saw_offset"], "kick offset emitted with axis off"
        assert out["peak_deg"] < 3.0, (
            f"frozen plant tilts {out['peak_deg']:.1f}° with the axis "
            f"OFF — baseline broken, not the axis")


def test_walk_kick_pulse_rocks_takeoff_then_dies_out():
    """With a 14° draw the pulse must roll the frozen stance into the
    hardware's measured takeoff band (>= 5°) AND fade: one second after
    the pulse window the excursion must be back under 4° — it is a
    roll-RATE injection, not a persistent lean."""
    peaks, afters = [], []
    for seed in SEEDS:
        out = _kick_rollout(seed, KICK_OVERRIDES)
        assert out["saw_offset"], "kick never fired at prob 1"
        peaks.append(out["peak_deg"])
        afters.append(out["peak_after_deg"])
    assert max(peaks) >= 5.0, (
        f"kick pulse barely moves the stance (peaks {peaks}) — the "
        f"fold→roll mapping does not reproduce the takeoff transient")
    assert max(afters) < 4.0, (
        f"excursion persists after the pulse (post-window peaks "
        f"{afters}) — supposed to be transient")


def test_walk_kick_never_touches_other_modes():
    """A rise-mode episode with the axis forced on must never see the
    pulse — the perturbation is scoped to walk episodes only."""
    cfg = load_config()
    for (sec, leaf), val in KICK_OVERRIDES.items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointGoalEnv(
        params=SimServoParams.from_cfg(None), randomize=True,
        dr_scale=0.0, episode_seconds=6.0, seed=0, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "rise" else 0.0)
    env.reset()
    assert env._ep_rand is not None \
        and env._ep_rand.walk_kick_roll_deg != 0.0, \
        "draw did not fire at prob 1 — test is vacuous"
    assert env._walk_kick_offset() is None, \
        "kick pulse leaked into a rise-mode episode"


# --------------------------------------------------------------------------
# WALK-PUSH bank (dr.walk_push_*, 08-12 — the takeoff mechanism the
# command-side kick could not deliver).
#
# replay_trace calibration (08-12): driving the sim plant with the
# recorded HARDWARE action streams produces the full 8-25° takeoff
# excursions, but the fold-pulse kick saturates at 5-10° peak /
# ~10 °/s at ANY dose — the planted opposite feet plus the write
# profile's rate limit eat the command. The push axis instead applies
# a signed half-sine roll TORQUE about the chassis's own x-axis via
# xfrc_applied over the first ~1.5 s of walk-mode episodes, bypassing
# the actuator path. Dose calibrated policy-in-the-loop (tip1 walking,
# 2.6 Nm / 1.5 s = hardware coin-flip regime; see domain_rand.py).
# The physical response NEEDS an active gait (a parked 6-foot plant
# absorbs 2.6 Nm at ~0.2°), so these tests pin the torque MECHANICS —
# fires in-window, dies at window end, xfrc row actually written and
# cleared, walk-mode only, and the shared-model shim exposing the
# schedule the MJX vec envs batch to the stepper — while the
# response-level match lives in the calibration probes and the
# device-side application in test_mjx_parity.py.

PUSH_OVERRIDES = {
    ("dr", "walk_push_prob"): 1.0,
    ("dr", "walk_push_nm"): "2.6,2.6",
    ("dr", "walk_push_s"): "1.5,1.5",
}


def _push_rollout(seed: int, overrides) -> dict:
    """Freeze at the start command and record the push-torque schedule
    plus the xfrc row actually seen by the physics loop."""
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv
    cfg = load_config()
    for (sec, leaf), val in overrides.items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None),
        randomize=(("dr", "walk_push_prob") in overrides),
        dr_scale=0.0, episode_seconds=6.0, seed=seed, cfg=cfg)
    gen = env._goal_gen
    gen.p_walk = 1.0
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 0.0)
    env.reset()
    q0 = env._cmd.copy()
    dur = (env._ep_rand.walk_push_dur_s if env._ep_rand is not None
           else 0.0)
    in_window_nm, after_nm, in_window_xfrc, after_xfrc = [], [], [], []
    for _ in range(int(3.0 / env.dt)):
        t = env._step_i * env.dt
        nm = env._walk_push_torque_nm()
        env.step(q_rad_to_action(q0))
        xfrc = float(np.linalg.norm(
            env.data.xfrc_applied[env._chassis_bid, 3:6]))
        if dur > 0.0 and t < dur:
            in_window_nm.append(nm)
            in_window_xfrc.append(xfrc)
        else:
            after_nm.append(nm)
            after_xfrc.append(xfrc)
    env.close()
    return {"in_nm": in_window_nm, "after_nm": after_nm,
            "in_xfrc": in_window_xfrc, "after_xfrc": after_xfrc}


def test_walk_push_default_off_is_inert():
    """No dr.walk_push override -> no draw, torque schedule identically
    zero and no xfrc ever written to the chassis row."""
    for seed in SEEDS:
        out = _push_rollout(seed, {})
        assert all(v == 0.0 for v in out["in_nm"] + out["after_nm"]), \
            "push torque emitted with the axis off"
        assert all(v == 0.0 for v in out["in_xfrc"] + out["after_xfrc"]), \
            "xfrc row written with the axis off"


def test_walk_push_torque_fires_then_dies_out():
    """Forced 2.6 Nm / 1.5 s draw: the half-sine must reach its peak
    inside the window (>= 2 Nm on both the schedule and the xfrc row
    the stepper saw) and be identically zero after the window — a
    roll-RATE injection, not a standing torque bias."""
    for seed in SEEDS:
        out = _push_rollout(seed, PUSH_OVERRIDES)
        peak = max(abs(v) for v in out["in_nm"])
        assert peak >= 2.0, (
            f"seed {seed}: push schedule peaks at {peak:.2f} Nm — the "
            f"forced 2.6 Nm draw never reached the physics loop")
        assert max(out["in_xfrc"]) >= 2.0, (
            f"seed {seed}: xfrc row peaks at {max(out['in_xfrc']):.2f} "
            f"— torque computed but never applied to the chassis")
        assert all(v == 0.0 for v in out["after_nm"]), \
            f"seed {seed}: push torque persists after the pulse window"
        assert all(v == 0.0 for v in out["after_xfrc"]), (
            f"seed {seed}: stale xfrc left on the chassis after the "
            f"window — state must not survive the pulse")


def test_walk_push_never_touches_other_modes():
    """A rise-mode episode with the axis forced on must never see the
    torque — the perturbation is scoped to walk episodes only."""
    cfg = load_config()
    for (sec, leaf), val in PUSH_OVERRIDES.items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointGoalEnv(
        params=SimServoParams.from_cfg(None), randomize=True,
        dr_scale=0.0, episode_seconds=6.0, seed=0, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "rise" else 0.0)
    env.reset()
    assert env._ep_rand is not None \
        and env._ep_rand.walk_push_peak_nm != 0.0, \
        "draw did not fire at prob 1 — test is vacuous"
    assert env._walk_push_torque_nm() == 0.0, \
        "push torque leaked into a rise-mode episode"


def test_walk_push_shared_model_shim_schedules_the_torque():
    """A shared-model (MJX shim) env with the axis forced on must
    construct, draw, and expose a nonzero per-tick torque schedule via
    _walk_push_torque_nm() — that is the value the MJX vec envs hand
    to the batched stepper's xfrc row every tick (plumbed 08-12; the
    device-side application is pinned in test_mjx_parity.py)."""
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv
    donor = SimHexapodJointGoalEnv(
        params=SimServoParams.from_cfg(None), randomize=False,
        dr_scale=0.0, episode_seconds=6.0, seed=0)
    cfg = load_config()
    for (sec, leaf), val in PUSH_OVERRIDES.items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=True,
        dr_scale=0.0, episode_seconds=6.0, seed=0, cfg=cfg,
        model=donor.model)
    gen = env._goal_gen
    gen.p_walk = 1.0
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 0.0)
    env.reset()
    assert env._ep_rand is not None \
        and env._ep_rand.walk_push_peak_nm != 0.0, \
        "shared-model shim never drew the push at prob 1"
    # Half-sine peak lands mid-window; sample the schedule there.
    env._step_i = int(round(
        env._ep_rand.walk_push_dur_s / 2.0 / env.dt))
    nm = env._walk_push_torque_nm()
    assert abs(nm) >= 2.0, (
        f"shim schedule peaks at {nm:.2f} Nm — the vec envs would "
        f"push nothing to the stepper")
    env.close()
    donor.close()


# --------------------------------------------------------------------------
# TRANS-DRAG bank — the stand/sit foot-scrape (operator 08-11 night).
#
# The failure this bank pins: NOTHING outside walk mode priced loaded
# foot-XY dragging — k_drag_loaded / k_drag_stance live in the
# walk-tick block only, so rise, lower, hold and every transition can
# scrape feet across the floor for free (the exact behavior the
# operator watches in the MuJoCo viewer and on the robot during stand
# and sit). reward.k_drag_trans charges the loaded slide beyond a
# per-episode allowance; allowances are MEASURED (probe 08-11): the
# demonstrated belly->plant rise inherently slides its pads 463 mm
# during the curl (rise/raise get a 0.55 m free budget), the quiet
# stand and the anchored-feet lower measure ~0.
#
# Required orderings at the tested operating point k=400/m:
#   quiet stand >> deliberate scraping (opposed yaw sweep, feet loaded
#   and sliding arcs) — the scrape must go NET NEGATIVE, not just lag;
#   the honest rise reference must keep byte-identical pay (its drag
#   sits inside the measured allowance);
#   the metric must see the scrape with the price OFF (metric and
#   price decoupled — the walk loadslip lesson).

TRANS_DRAG_K = 400.0


def _tdrag_hold_rollout(policy: str, seed: int, k: float) -> dict:
    """quiet   hold the settled plant exactly (the intended skill).
    scrape    left tripod yaws +, right tripod yaws −, ±10° at 0.5 Hz:
              the body cannot counter-rotate so all six LOADED feet
              grind arcs on the floor (~1.25 m/episode measured) while
              the torso stays level at height — the pure form of the
              stand/sit scrape, priced by nothing else in the stack
              (feet never leave the ground: hold_still_gate's feet
              factor stays 1, and only its stillness Gaussian nicks
              the sweep)."""
    ov = dict(HOLD_OVERRIDES)
    ov[("reward", "k_drag_trans")] = k
    env = _make_hold_env(seed, ov)
    env.reset()
    q0 = env.data.qpos[env._qadr].copy()
    tot, drag_mm, charge, step = 0.0, 0.0, 0.0, 0
    while True:
        q = q0.copy()
        if policy == "scrape":
            s = 10.0 * DEG2RAD * math.sin(
                2.0 * math.pi * 0.5 * step * env.dt)
            for leg in range(6):
                q[3 * leg] += s if leg < 3 else -s
        _obs, r, term, trunc, info = env.step(q_rad_to_action(q))
        tot += float(r)
        drag_mm += float(info.get("trans_drag_mm", 0.0))
        charge += float(info.get("reward_drag_trans", 0.0))
        step += 1
        if term or trunc:
            break
    env.close()
    return {"ret": tot, "drag_mm": drag_mm, "charge": charge}


def _tdrag_rise_rollout(seed: int, k: float) -> dict:
    """The demonstrated belly->plant reference under the rise stack
    with the drag charge on/off."""
    ov = dict(RISE_OVERRIDES)
    ov[("reward", "k_drag_trans")] = k
    env = _make_rise_env(seed, ov)
    env.reset()
    ref = np.load(ROOT / RISE_REF)
    q_ref, ramp_ref = ref["q_rad"], int(ref["ramp_i0"])
    tot, drag_mm, charge, step = 0.0, 0.0, 0.0, 0
    while True:
        j = ramp_ref + (step - env._rise_ramp_i0)
        act = q_rad_to_action(q_ref[min(max(j, 0), len(q_ref) - 1)])
        _obs, r, term, trunc, info = env.step(act)
        tot += float(r)
        drag_mm += float(info.get("trans_drag_mm", 0.0))
        charge += float(info.get("reward_drag_trans", 0.0))
        step += 1
        if term or trunc:
            break
    env.close()
    return {"ret": tot, "drag_mm": drag_mm, "charge": charge}


@pytest.fixture(scope="module")
def tdrag_bank() -> dict:
    return {
        "on": {p: [_tdrag_hold_rollout(p, s, TRANS_DRAG_K)
                   for s in SEEDS] for p in ("quiet", "scrape")},
        "off": {p: [_tdrag_hold_rollout(p, s, 0.0)
                    for s in SEEDS] for p in ("quiet", "scrape")},
        "rise_on": [_tdrag_rise_rollout(s, TRANS_DRAG_K) for s in SEEDS],
        "rise_off": [_tdrag_rise_rollout(s, 0.0) for s in SEEDS],
    }


def test_trans_drag_metric_sees_the_scrape_with_price_off(tdrag_bank):
    """trans_drag_mm must discriminate scraping from a quiet stand
    even at k=0 — the metric is the eval/W&B triage surface and must
    never be coupled to the price (walk loadslip lesson)."""
    for r in tdrag_bank["off"]["scrape"]:
        assert r["drag_mm"] > 700.0, (
            f"scripted scraper only measures {r['drag_mm']:.0f}mm of "
            f"loaded drag — the reference behavior is broken")
        assert r["charge"] == 0.0, "charge fired with k=0"
    for r in tdrag_bank["off"]["quiet"]:
        assert r["drag_mm"] < 30.0, (
            f"quiet stand measures {r['drag_mm']:.0f}mm of loaded "
            f"drag — deadband miscalibrated")


def test_trans_drag_charge_bites_the_scraper(tdrag_bank):
    """At the operating point the scrape must pay a real price (its
    ~1.25 m of loaded slide × k=400) and go decisively below the quiet
    stand — scraping must be a net-negative strategy, not a discount."""
    for r in tdrag_bank["on"]["scrape"]:
        assert r["charge"] < -350.0, (
            f"drag charge {r['charge']:.0f} barely prices the scrape")
    q = float(np.mean([r["ret"] for r in tdrag_bank["on"]["quiet"]]))
    s = float(np.mean([r["ret"] for r in tdrag_bank["on"]["scrape"]]))
    assert q > s + 300.0, (
        f"scraping rivals the quiet stand under k_drag_trans: "
        f"quiet {q:+.0f} vs scrape {s:+.0f}")


def test_trans_drag_never_taxes_the_quiet_stand(tdrag_bank):
    """The honest quiet stand slides nothing — its charge must be
    exactly zero and its return unchanged by the axis."""
    for r_on, r_off in zip(tdrag_bank["on"]["quiet"],
                           tdrag_bank["off"]["quiet"]):
        assert r_on["charge"] == 0.0, (
            f"quiet stand charged {r_on['charge']:.2f}")
        assert abs(r_on["ret"] - r_off["ret"]) < 1e-6, (
            f"axis changes the quiet stand's return with zero drag: "
            f"{r_on['ret']:.2f} vs {r_off['ret']:.2f}")


def test_trans_drag_honest_rise_keeps_full_pay(tdrag_bank):
    """The demonstrated rise slides its pads ~463 mm during the curl —
    inherent to the belly->plant path, not a cheat. The measured 0.55 m
    rise allowance must keep the reference completely uncharged, so
    the charge can ride into mixed-mode stand arms without repricing
    the one behavior that tape-provably works."""
    for r_on, r_off in zip(tdrag_bank["rise_on"], tdrag_bank["rise_off"]):
        assert 350.0 < r_on["drag_mm"] < 550.0, (
            f"rise reference drag {r_on['drag_mm']:.0f}mm left the "
            f"measured band — re-measure before trusting the allowance")
        assert r_on["charge"] == 0.0, (
            f"honest rise charged {r_on['charge']:.2f} — allowance "
            f"too tight")
        assert abs(r_on["ret"] - r_off["ret"]) < 1e-6, (
            f"axis changes the honest rise's return: "
            f"{r_on['ret']:.2f} vs {r_off['ret']:.2f}")


# --------------------------------------------------------------------------
# GETUP bank — the unified recover→stand→walk mode (08-11 from-scratch
# reward redesign, REWARD.md §4b). Every stand-campaign exploit is
# re-banked under the NEW stack before its first discovery run:
#
#   freeze    a level belly-rest is level — under the legacy kernel it
#             farms ~1/tick; here it must earn ~nothing (kernel income
#             is stripped on getup ticks, the ratchet seeds at spawn).
#   flagleg   the cheat that beat SEVEN rise mechanisms: five legs
#             rise, one stays flagged straight out. Priced by the
#             measured-load f_feet, the pad-spread f_flag fade, and
#             the S^3 hold gate.
#   stilt     hip 0 / knee 80 pop. Measured in-sim (correct zeros):
#             this settles into a NARROW crouch-stand (z 118 mm,
#             below the plant, all six loaded) — a partial stand, not
#             the hardware overshoot disaster. It stands in ~2 s and
#             farms hold pay for the rest of the horizon, so its
#             EPISODE total legitimately beats a 13 s honest rise;
#             the honest comparison (what the policy optimizes at
#             steady state) is per-tick income, banked as `tail`
#             below. The fp/height fades must hold its tick pay to
#             scraps of the plant stand's.
#   shuffle   locomotion without standing (the belly-shuffle the
#             champion handoff collapses into) — must earn ~0 through
#             the f_height gate no matter how much it progresses.
#   park      holding the plant through a move command — the built-in
#             progress gate must pay it ~0 walk income.

GETUP_OVERRIDES = {
    # Falls are recoverable states in this mode; the run trains with a
    # widened trip. Bank under the same envelope.
    ("safety", "max_roll_deg"): 60.0,
    ("safety", "max_pitch_deg"): 60.0,
}


def _make_getup_env(seed: int, *, start: str, cmd: tuple[float, float],
                    overrides: dict | None = None):
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv

    cfg = load_config()
    for (sec, leaf), val in (overrides or GETUP_OVERRIDES).items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=False,
        dr_scale=0.0, episode_seconds=16.0, seed=seed, cfg=cfg)
    gen = env._goal_gen
    for attr in [a for a in vars(gen) if a.startswith("p_")]:
        setattr(gen, attr, 0.0)
    gen.p_getup = 1.0
    env.force_getup_start = start
    env.force_getup_cmd = cmd
    return env


def test_getup_forward_only_commands():
    """goal.getup_forward_only=1 (RISE_WALK_NEXT_48H P1 unified task):
    the sampled command schedule is forward-or-stop ONLY (vy == 0,
    vx >= 0 everywhere), and the discarded angle draws keep the rng
    stream seed-identical to the full task — same start kind and the
    same stop/go segment mask. (Speed magnitudes match only at
    plateaus: blends interpolate vx/vy component-wise, so the full
    task's |v| is sub-linear mid-blend — not asserted.)"""
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv

    def make(fwd_only: int, seed: int):
        cfg = load_config()
        for (sec, leaf), val in GETUP_OVERRIDES.items():
            cfg.setdefault(sec, {})[leaf] = val
        cfg.setdefault("goal", {})["getup_forward_only"] = fwd_only
        env = SimHexapodJointWalkEnv(
            params=SimServoParams.from_cfg(None), randomize=False,
            dr_scale=0.0, episode_seconds=30.0, seed=seed, cfg=cfg)
        gen = env._goal_gen
        for attr in [a for a in vars(gen) if a.startswith("p_")]:
            setattr(gen, attr, 0.0)
        gen.p_getup = 1.0
        return env

    for seed in (11, 12, 13):
        env_f = make(1, seed)
        env_full = make(0, seed)
        env_f.reset(seed=seed)
        env_full.reset(seed=seed)
        tf, tu = env_f._goal_traj, env_full._goal_traj
        assert np.all(tf.vy == 0.0), "forward-only drew a lateral cmd"
        assert np.all(tf.vx >= -1e-12), "forward-only drew a reverse cmd"
        # rng-stream parity with the full task under the same seed
        assert tf.start_kind == tu.start_kind
        assert np.array_equal(tf.vx != 0.0, tu.vx != 0.0)
        env_f.close()
        env_full.close()


def _getup_rollout(policy: str, seed: int, *, start: str = "zero",
                   cmd: tuple[float, float] = (0.0, 0.0)) -> dict:
    from tripod_gait import TripodGait

    env = _make_getup_env(seed, start=start, cmd=cmd)
    env.reset()
    assert getattr(env, "_is_getup", False), "getup mode did not arm"
    ref = np.load(ROOT / RISE_REF)
    q_ref = ref["q_rad"]
    n_ref = len(q_ref)
    # "partial" = honest work that stops 40% into the ramp: a held
    # LOW CROUCH-STAND (z ~57 mm, feet carrying full weight) — the
    # first waypoint where the pipeline banks anything. The crouch
    # itself (ramp_i0) still has its feet in the AIR (bank-measured
    # 0 N of touch through the whole curl), and freezing further up
    # the ramp just sags and slides.
    r_i0 = int(ref["ramp_i0"])
    j_part = r_i0 + int((n_ref - r_i0) * 0.4)
    q0 = env.data.qpos[env._qadr].copy()
    q_stilt = np.array([0.0, 0.0, 80.0] * 6) * DEG2RAD
    plant_rad = np.array([0.0, *WALK_PLANT] * 6) * DEG2RAD
    gait = TripodGait(vx=0.0)
    gait.sync_plant_stance(*(WALK_PLANT if policy != "shuffle"
                             else (0.0, 15.0)))
    gait.reset_phase()
    rng = np.random.default_rng(seed)

    total, step, best, terminated = 0.0, 0, 0.0, False
    rewards: list[float] = []
    while True:
        t = step * env.dt
        if policy == "replay":
            act = q_rad_to_action(q_ref[min(step, n_ref - 1)])
        elif policy == "partial":
            act = q_rad_to_action(q_ref[min(step, j_part)])
        elif policy == "flagleg":
            q = q_ref[min(step, n_ref - 1)].copy()
            q[0:3] = q0[0:3]
            act = q_rad_to_action(q)
        elif policy == "freeze":
            act = q_rad_to_action(q0)
        elif policy == "stilt":
            act = q_rad_to_action(q0 if t < 2.0 else q_stilt)
        elif policy == "thrash":
            act = rng.uniform(-1.0, 1.0, size=18)
        elif policy in ("gait", "shuffle"):
            g = env._current_goal()
            gait.set_velocity(vx=float(getattr(g, "vx_ref", 0.0)),
                              vy=float(getattr(g, "vy_ref", 0.0)))
            act = q_rad_to_action(
                np.asarray(gait.desired_deg(t)) * DEG2RAD)
        else:  # "park": hold the plant, refuse the command
            act = q_rad_to_action(plant_rad)
        _obs, r, term, trunc, info = env.step(act)
        total += float(r)
        rewards.append(float(r))
        best = float(info.get("getup_best", best))
        step += 1
        if term or trunc:
            terminated = term
            break
    env.close()
    # Steady-state per-tick income (last 2 s): what a converged policy
    # actually optimizes once its behavior settles — total return over
    # a fixed horizon structurally favors whoever stands FASTEST.
    tail = float(np.mean(rewards[-50:])) if rewards else 0.0
    return {"ret": total, "best": best, "tail": tail,
            "terminated": terminated}


GETUP_STAND_POLICIES = ("replay", "partial", "flagleg", "stilt",
                        "freeze", "thrash")


@pytest.fixture(scope="module")
def getup_stand_bank() -> dict[str, list[dict]]:
    """Zero command all episode, belly-zero start: the recover+stand
    half of the task in isolation."""
    return {p: [_getup_rollout(p, s) for s in SEEDS]
            for p in GETUP_STAND_POLICIES}


@pytest.fixture(scope="module")
def getup_stand_returns(getup_stand_bank) -> dict[str, float]:
    return {p: float(np.mean([r["ret"] for r in rolls]))
            for p, rolls in getup_stand_bank.items()}


def test_getup_replay_dominates_all_cheats(getup_stand_returns):
    """The demonstrated belly->plant rise must out-earn every known
    non-standing cheat by the campaign's standard margin (>=2x and
    +50). The stilt (a fast PARTIAL stand) is priced per-tick in
    test_getup_stilt_pays_scraps_per_tick instead — episode totals
    structurally reward standing fast, which is desired."""
    replay = getup_stand_returns["replay"]
    best_cheat = max(getup_stand_returns["freeze"],
                     getup_stand_returns["flagleg"],
                     getup_stand_returns["thrash"])
    assert replay > 2.0 * best_cheat and replay > best_cheat + 50.0, (
        f"getup stack prefers a known cheat: {getup_stand_returns} — "
        "fix the reward BEFORE launching any getup arm.")


def test_getup_freeze_earns_nothing(getup_stand_returns):
    """The ratchet seeds at the spawn posture and the kernel is
    stripped, so refusal must earn ~0 BY CONSTRUCTION (small negative
    regularizers), never a living."""
    assert getup_stand_returns["freeze"] < 1.0, (
        f"freezing earns {getup_stand_returns['freeze']:+.1f} under "
        "the getup stack — an alive bonus crept back in.")


def test_getup_honest_ordering(getup_stand_returns):
    """replay > partial(crouch) > refusal > thrash — the gradient
    toward trying honestly must exist at every rung. (flag-leg is NOT
    on this ladder: five loaded legs at height genuinely banks most of
    the one-shot pipeline — that slope toward fixing the sixth leg is
    wanted. Its discrimination is steady income, tested below.)"""
    assert (getup_stand_returns["replay"]
            > getup_stand_returns["partial"] + 50.0), getup_stand_returns
    assert (getup_stand_returns["partial"]
            > getup_stand_returns["freeze"]), (
        f"banking the crouch pays no better than refusing to move: "
        f"{getup_stand_returns}")
    assert (getup_stand_returns["partial"]
            > getup_stand_returns["thrash"] + 20.0), (
        f"thrashing rivals honest partial progress: "
        f"{getup_stand_returns}")


def test_getup_stilt_pays_scraps_per_tick(getup_stand_bank):
    """The narrow hip-0 stand must earn well under half the plant
    stand's steady per-tick income (the footprint + height fades) —
    a downhill-sloped scrap, so the gradient points at the plant, but
    never a rival living. flag-leg's steady income must be ~nothing
    (S^3 through the load + spread fades)."""
    tails = {p: float(np.mean([r["tail"] for r in getup_stand_bank[p]]))
             for p in ("replay", "stilt", "flagleg")}
    assert tails["replay"] > 2.0 * tails["stilt"], (
        f"stilt stand rivals the plant stand per tick: {tails}")
    assert tails["flagleg"] < 0.15 * tails["replay"], (
        f"flag-leg keeps a steady living: {tails}")


def test_getup_flagleg_earns_scraps(getup_stand_returns):
    """The seven-mechanism cheat: under measured-load f_feet + the
    pad-spread f_flag fade + the S^3 hold gate it must keep at most
    scraps of the honest pay (fade, so a slope exists — holdstill1
    lesson — but never a consolation income)."""
    assert (getup_stand_returns["flagleg"]
            < 0.35 * getup_stand_returns["replay"]), (
        f"flag-leg still collects real income: {getup_stand_returns}")


def test_getup_ratchet_reaches_the_stand(getup_stand_bank):
    """Mechanism health: the honest replay must drive the staged
    ratchet close to 1.0 (the income actually pays the pipeline), and
    the freeze must stay pinned at its seeded spawn value."""
    for r in getup_stand_bank["replay"]:
        assert r["best"] >= 0.80, (
            f"honest rise only ratchets to {r['best']:.2f} — the "
            "staged potential never reaches the stand it should pay.")
    for r in getup_stand_bank["freeze"]:
        assert r["best"] <= 0.30, (
            f"freeze ratchets to {r['best']:.2f} without moving — "
            "the spawn posture is being paid as progress.")


@pytest.fixture(scope="module")
def getup_walk_returns() -> dict[str, float]:
    """Forward command from a plant start: the walk half of the task.
    All three policies bank the same (near-zero) ratchet income at the
    plant spawn, so the separation is pure gated walk income."""
    return {p: float(np.mean([
        _getup_rollout(p, s, start="plant", cmd=(WALK_CMD_VX, 0.0))["ret"]
        for s in SEEDS]))
        for p in ("gait", "shuffle", "park")}


def test_getup_gait_dominates_park_and_shuffle(getup_walk_returns):
    """The hardware-proven tripod gait must decisively out-earn both
    refusal (park) and locomotion-without-standing (belly shuffle)."""
    gait = getup_walk_returns["gait"]
    assert gait > 2.0 * getup_walk_returns["park"] + 50.0, (
        f"parking through a move command rivals walking: "
        f"{getup_walk_returns}")
    assert gait > 2.0 * getup_walk_returns["shuffle"] + 50.0, (
        f"the belly shuffle rivals honest walking: {getup_walk_returns}")


def test_getup_shuffle_earns_scraps(getup_walk_returns):
    """The structural claim of the S gate: progress made below the
    supported-stand height is worth ~nothing, no matter how much
    ground it covers — the exact behavior the champion handoff
    collapses into must never be a paid basin."""
    assert (getup_walk_returns["shuffle"]
            < 0.25 * getup_walk_returns["gait"]), (
        f"belly-shuffle keeps a real fraction of walk income: "
        f"{getup_walk_returns}")


# --------------------------------------------------------------------------
# GOAL-PROFILE JITTER bank — goal.rise_ramp_jitter / goal.lower_ramp_jitter.
#
# Model tour, 08-11 (rl_docs/MODEL_TOUR_2026-08-11.md): the deployed
# stance checkpoint passes its training-profile gates yet, under
# play.py's interactive ramp — SAME height targets, slightly different
# ramp shape — its belly rise stalls at 55 mm forever and its sit from
# the 142 mm plant tips over at ~2.5 s, deterministically. The policy
# memorized ONE ramp choreography. The jitter axis randomizes each
# episode's ramp duration so the trained skill generalizes across
# profile shapes; rl_move.sim.eval_session is the deployment-side gate.
# These are pure sampler tests (no sim rollout — fast).

def _ramp_len(height: np.ndarray) -> int:
    """Ticks the height schedule spends strictly between 0 and target."""
    tgt = height[int(np.argmax(np.abs(height)))]
    mid = (np.abs(height) > 1e-12) & (np.abs(height - tgt) > 1e-12)
    return int(mid.sum())


_PURE_MIX = {"p_hold": 0.0, "p_lean": 0.0, "p_track": 0.0,
             "p_unload": 0.0, "p_raise": 0.0, "p_rise": 0.0,
             "p_lower": 0.0}


def _gen(goal_cfg: dict):
    from rl_move.sim.goal_task import GoalGenerator
    # Trained runs carry actions.max_height_mm ~ 80; the bare default
    # (5 mm) would squash the rise/lower bands and hide the mechanism.
    # _PURE_MIX zeroes every default mode weight so p_rise/p_lower=1.0
    # in the caller's cfg really means a pure mix.
    return GoalGenerator({"goal": {**_PURE_MIX, **goal_cfg},
                          "actions": {"max_height_mm": 80.0}})


def _profiles(goal_cfg: dict, seed: int = 7, n: int = 30) -> list:
    gen = _gen(goal_cfg)
    rng = np.random.default_rng(seed)
    return [gen.sample(rng, 400, 0.04) for _ in range(n)]


def test_ramp_jitter_default_off_is_bit_exact():
    """Key absent and explicit 0.0 must produce identical schedules from
    the same seed — the conditional draw never touches the rng stream
    when the axis is off (house rng discipline)."""
    for mode_cfg in ({"p_rise": 1.0}, {"p_lower": 1.0}):
        base = _profiles(dict(mode_cfg))
        off = _profiles({**mode_cfg, "rise_ramp_jitter": 0.0,
                         "lower_ramp_jitter": 0.0})
        for a, b in zip(base, off):
            assert np.array_equal(a.height, b.height), (
                "jitter=0 shifted the sampled schedule")


def test_ramp_jitter_varies_the_rise_ramp():
    """With jitter on, rise episodes must draw genuinely different ramp
    durations, all inside the (1 +- j) band of the base 4 s ramp."""
    profs = _profiles({"p_rise": 1.0, "rise_ramp_jitter": 0.5})
    lens = {_ramp_len(p.height) for p in profs}
    assert len(lens) >= 8, f"rise ramps barely vary: {sorted(lens)}"
    base_n = 4.0 / 0.04
    assert all(0.45 * base_n <= n <= 1.55 * base_n for n in lens), (
        f"rise ramp outside the jitter band: {sorted(lens)}")


def test_ramp_jitter_varies_the_lower_ramp():
    """Same contract for lower episodes (base 5 s ramp)."""
    profs = [p for p in _profiles({"p_lower": 1.0,
                                   "lower_ramp_jitter": 0.5})
             if float(np.min(p.height)) < 0.0]   # skip belly-start flats
    lens = {_ramp_len(p.height) for p in profs}
    assert len(lens) >= 8, f"lower ramps barely vary: {sorted(lens)}"
    base_n = 5.0 / 0.04
    assert all(0.45 * base_n <= n <= 1.55 * base_n for n in lens), (
        f"lower ramp outside the jitter band: {sorted(lens)}")


def test_ramp_jitter_never_touches_targets_or_holds():
    """Jitter changes ramp DURATION only: the height target band and the
    pre-ramp hold length stay exactly as configured."""
    plain = _profiles({"p_lower": 1.0})
    jit = _profiles({"p_lower": 1.0, "lower_ramp_jitter": 0.5})
    hold_n = max(1, int(round(1.0 / 0.04)))   # lower_hold_s default
    for p in jit:
        if float(np.min(p.height)) >= 0.0:
            continue
        tgt_mm = -float(np.min(p.height)) * 1000
        assert 25.0 - 1e-6 <= tgt_mm <= 55.0 + 1e-6, (
            f"lower target left the configured band: {tgt_mm}")
        assert np.all(p.height[:hold_n] == 0.0), (
            "jitter ate into the pre-ramp hold window")
    # And the plain generator still produces in-band targets (sanity).
    for p in plain:
        if float(np.min(p.height)) < 0.0:
            assert -0.055 - 1e-9 <= float(np.min(p.height)) <= -0.025 + 1e-9


# --------------------------------------------------------------------------
# SUPPORT-MARGIN bank (reward.k_support_margin > 0 — the stand knife-edge
# lever, 08-12). Replay of all ten 08-11 hardware stand-failure tapes
# (replay_trace.py + a support-polygon trace) found the real trip
# mechanism: in the last ~1 s of the deployed rise the policy's own
# commands degenerate the support to THREE feet (L0/L1/L4) with the CoM
# margin flickering +25/-25 mm every tick — sim survives on a
# hair-trigger catch, hardware tips on the L4 pivot. The (long-built,
# never-trained) k_support_margin term prices exactly this: income
# proportional to CoM depth inside the loaded-feet polygon, capped at
# 40 mm, exempt below 3 loaded feet (belly rest). These tests pin
# (a) k=0 is bit-exact (default-off), (b) a wide loaded stance earns
# the cap, (c) the REPLAY'S OWN degenerate support set (standing on
# legs 0/1/4 only) earns a small fraction of the wide stance — the
# gradient pushes toward keeping feet down and the CoM deep, and no
# cheat with fewer/asymmetric feet out-earns the honest plant.

MARGIN_OVERRIDES = dict(HOLD_LOAD_OVERRIDES)
MARGIN_OVERRIDES[("reward", "k_support_margin")] = 1.0


def _margin_rollout(seed: int, overrides, lift_legs=()) -> float:
    """Scripted stance hold with the given legs lifted; returns the
    episode return (same recipe as _hold_load_rollout, arbitrary legs)."""
    env = _make_hold_env(seed, overrides)
    env.reset()
    q = env.data.qpos[env._qadr].copy()
    for leg in lift_legs:
        q[3 * leg + 1] -= HOVER_LIFT_DEG * DEG2RAD
    act = q_rad_to_action(q)
    total = 0.0
    while True:
        _obs, r, term, trunc, _info = env.step(act)
        total += float(r)
        if term or trunc:
            assert not term, f"margin bank pose lift={lift_legs} fell over"
            break
    env.close()
    return total


def test_support_margin_default_off_bit_exact():
    """k_support_margin=0 (explicit) must equal the key being absent."""
    off = dict(HOLD_LOAD_OVERRIDES)
    off[("reward", "k_support_margin")] = 0.0
    a = _margin_rollout(SEEDS[0], HOLD_LOAD_OVERRIDES)
    b = _margin_rollout(SEEDS[0], off)
    assert a == b, f"k_support_margin=0 changed the reward path ({a} vs {b})"


def test_support_margin_pays_wide_stance_the_cap():
    """A quiet six-foot plant stance sits >=40 mm deep: the margin
    income is the full cap every tick (measured 375.0 = k*steps)."""
    for s in SEEDS[:2]:
        inc = (_margin_rollout(s, MARGIN_OVERRIDES)
               - _margin_rollout(s, HOLD_LOAD_OVERRIDES))
        assert inc > 350.0, f"wide stance margin income only {inc:.1f}"


def test_support_margin_prices_the_replay_knife_edge():
    """Standing on legs 0/1/4 only — the EXACT degenerate support set
    from the hardware stand-failure replays — must earn a small
    fraction of the wide stance's margin income (measured 70.6 vs
    375.0, 0.19x): the term's gradient points from the knife edge back
    toward the honest plant, and shedding feet never pays."""
    for s in SEEDS[:2]:
        inc_wide = (_margin_rollout(s, MARGIN_OVERRIDES)
                    - _margin_rollout(s, HOLD_LOAD_OVERRIDES))
        inc_knife = (_margin_rollout(s, MARGIN_OVERRIDES, (2, 3, 5))
                     - _margin_rollout(s, HOLD_LOAD_OVERRIDES, (2, 3, 5)))
        assert inc_knife < 0.3 * inc_wide, (
            f"degenerate 0/1/4 support still earns {inc_knife:.1f} vs "
            f"wide {inc_wide:.1f} — margin term not discriminating")
        assert inc_knife >= 0.0, "margin income went negative on a " \
            "stable tripod — cap/exemption broken"


# --------------------------------------------------------------------------
# QUADWALK bank — commanded walking on the four support legs with the
# front pair (0, 5) lifted (quad track "four-leg WALKING" spec, 08-13).
# Stack = the walk-lineage champion stack (WALK_OVERRIDES — the exact
# stack the quad-hold lineage trained with) + the quad clear/plant
# income (cw-quad-hold2 coefficients). Required ordering:
#
#     honest rear-four gait (fronts up) > six-leg walk (fronts
#     stepping) > fronts-down drag / freeze-at-quad-stance
#
# The 08-13 audit found the pre-quadwalk stack REWARD-PUNISHED and
# EVAL-INVALIDATED any honest quad walk (k_park_duty's window spanned
# the commanded-lifted fronts ~0.2k/tick; eval sacrificed_legs counted
# them too). These tests pin that the mode's lift-leg exemptions plus
# the clear income actually price the task as specified — BEFORE any
# PPO arm trains on it.

QUADWALK_OVERRIDES = dict(WALK_OVERRIDES)
QUADWALK_OVERRIDES.update({
    ("reward", "k_quad_clear"): 1.5,
    ("reward", "k_quad_plant"): 1.0,
    ("goal", "quad_grace_s"): 1.5,
    ("goal", "quadwalk_speed_min_m_s"): 0.03,
    ("goal", "quadwalk_speed_max_m_s"): 0.04,
})
QUADWALK_CMD_VX = 0.03      # slower than walk: four feet, small polygon
# Raised front pose (yaw, hip, knee) rad — quadruped_feasibility
# FRONT_POSES["tuck"], the feasibility-GO claw pose.
QW_TUCK_RAD = (0.0, -1.10, 2.40)
QW_LIFT = (0, 5)


def _make_quadwalk_env(seed: int, overrides: dict | None = None):
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv

    cfg = load_config()
    for (sec, leaf), val in (overrides or QUADWALK_OVERRIDES).items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=False,
        dr_scale=0.0, episode_seconds=15.0, seed=seed, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad", "walk", "quadwalk"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "quadwalk" else 0.0)
    return env


def _quadwalk_rollout(policy: str, seed: int, *,
                      vx: float = QUADWALK_CMD_VX,
                      overrides: dict | None = None) -> dict:
    """One 15 s quadwalk episode under a scripted policy.

    quadgait   rear-four tripod-derived trot (legs 2,4 / 1,3 pairs),
               fronts held at the tuck pose — the honest behavior
    sixleg     plain six-leg tripod gait — fronts stepping (cheat)
    frontdrag  rear-four gait, fronts held at PLANT angles: feet down,
               dragging as the body moves (cheat)
    freeze     quad stance park: support legs planted, fronts tucked,
               no stepping (refusal)
    """
    from tripod_gait import TripodGait

    env = _make_quadwalk_env(seed, overrides)
    env.reset()
    traj = env._goal_traj
    assert getattr(traj, "mode", "") == "quadwalk", (
        f"quadwalk forcing broken: sampled {getattr(traj, 'mode', '?')}")
    n = len(traj.vx)
    hold_n = int(round(2.0 / env.dt))
    ramp_n = int(round(1.0 / env.dt))
    ramp = np.linspace(0.0, 1.0, ramp_n)
    traj.vx[:] = vx
    traj.vx[:hold_n] = 0.0
    traj.vx[hold_n:hold_n + ramp_n] = vx * ramp
    traj.vy[:] = 0.0
    if traj.wz is not None:
        traj.wz[:] = 0.0

    gait = TripodGait(vx=0.0, lift=0.025)
    gait.sync_plant_stance(*WALK_PLANT)
    if policy in ("quadgait", "freeze"):
        # Probe-derived viable statics (08-13): without the mid-leg
        # forward splay the tucked stance pitch-trips in <1 s (CoM
        # ahead of the 4-foot polygon front edge); the tuck must also
        # blend in over ~1.5 s. NOTE the splayed trot still does not
        # TRANSLATE — see QUADWALK_REFERENCE_BLOCKED; the quadgait
        # branch must be replaced by a working scheme (probe_quad_
        # crawl.py) before the ordering fixture is un-skipped.
        _orig_target = gait._foot_target_in_body

        def _splayed(i, vx, vy, om):
            dx, dy, dz = _orig_target(i, vx, vy, om)
            if i in (1, 4):
                dx += 0.06
            return (dx, dy, dz)
        gait._foot_target_in_body = _splayed
    plant_rad = np.array([0.0, *WALK_PLANT] * 6) * DEG2RAD
    gait.reset_phase()
    tuck = np.array(QW_TUCK_RAD)

    total, step = 0.0, 0
    lift_contact, lift_ticks = 0, 0
    n_skip = int(round(3.0 / env.dt))
    while True:
        t = step * env.dt
        i = min(step, n - 1)
        gait.set_velocity(
            vx=0.0 if policy == "freeze" else float(traj.vx[i]),
            vy=0.0 if policy == "freeze" else float(traj.vy[i]))
        q = np.asarray(gait.desired_deg(t)) * DEG2RAD
        if policy in ("quadgait", "freeze"):
            a = min(t / 1.5, 1.0)
            for leg in QW_LIFT:
                q[3 * leg:3 * leg + 3] = (
                    (1 - a) * plant_rad[3 * leg:3 * leg + 3] + a * tuck)
        elif policy == "frontdrag":
            for leg in QW_LIFT:
                q[3 * leg:3 * leg + 3] = plant_rad[3 * leg:3 * leg + 3]
        # (sixleg: q untouched — fronts step like any other leg)
        _obs, r, term, trunc, _info = env.step(q_rad_to_action(q))
        total += float(r)
        if step >= n_skip:
            lift_ticks += 1
            lift_contact += sum(
                1 for leg in QW_LIFT
                if float(env.data.sensordata[env._touch_adr[leg]]) > 0.5)
        step += 1
        if term or trunc:
            break
    env.close()
    return {"return": total, "terminated": bool(term),
            "lift_duty_tail": (lift_contact / (2.0 * lift_ticks)
                               if lift_ticks else 1.0)}


# HONEST-REFERENCE STATUS (08-13, spec cycle finding): NO open-loop
# scripted quad gait walks on this platform in sim yet. Tried, all
# failing to translate (probe_quad_crawl.py reproduces every scheme):
#   - rear-four 2-2 trot (tripod timing minus fronts): stable, fronts
#     clean, fwd ~0.00 m/15 s (diagonal 2-leg support pivots/slips);
#   - 4-beat crawl duty 0.75: mid legs PINNED under load (CoM outside
#     the mid-swing support triangle — matches the feasibility sweep
#     geometry: GO rows all splay the mid feet forward);
#   - + mid splay 0.04-0.06 m & body-back 0.05-0.06 m (the sweep's GO
#     statics): mids still pinned;
#   - + lateral sway 0.05-0.06 m (leading or in-phase), lift-first or
#     delayed swings, periods 2.0-4.0 s, vx 0.02-0.04: mids lift but
#     the body drifts -0.02..-0.10 m (slip/rock consumes everything;
#     rear legs chatter 2-3x their commanded step count).
# Static hold margin (quadruped_feasibility GO) does NOT extend to
# open-loop stepping. Until an honest reference EXISTS (a video-driven
# scripted-crawl iteration on a train pod, or an operator ruling on
# relaxing the reference source), the ordering tests below SKIP —
# which keeps every quadwalk PPO arm launch-blocked by MDP_PREFLIGHT,
# exactly as the rules intend. The exemption/inertness tests further
# down run regardless.
QUADWALK_REFERENCE_BLOCKED = (
    "QUADWALK ordering bank skipped: no accepted reference gait yet. "
    "Scripted open-loop rear-four gaits are measured geometrically "
    "infeasible (probe_quad_crawl.py --diag, quad/STATUS.md 08-13); "
    "per the operator ruling of 08-13 ~12:4x UTC a feedback/RL "
    "rear-four-stepping policy MAY become the reference, but ONLY "
    "after passing the pre-registered robustness gate in "
    "rl_docs/tracks/quad/QUADWALK_REF_GATE.md. Until a candidate "
    "passes that gate these ordering tests keep skipping and no "
    "scripted-bank claim may cite an RL reference. Quadwalk training "
    "arms are launchable under the ruling's terms despite this skip.")


@pytest.fixture(scope="module")
def quadwalk_bank() -> dict[str, list[dict]]:
    pytest.skip(QUADWALK_REFERENCE_BLOCKED)
    return {p: [_quadwalk_rollout(p, s) for s in SEEDS[:2]]
            for p in ("quadgait", "sixleg", "frontdrag", "freeze")}


def _qw_mean(bank, policy):
    return float(np.mean([r["return"] for r in bank[policy]]))


def test_quadwalk_honest_gait_survives(quadwalk_bank):
    """The honest reference must be PHYSICALLY viable: the scripted
    rear-four gait with tucked fronts finishes its episodes upright
    (feasibility GO was static; this pins the walking form) and its
    fronts genuinely stay up (tail duty < 0.15 — the same criterion
    the eval harness scores fronts_lifted with)."""
    for r in quadwalk_bank["quadgait"]:
        assert not r["terminated"], (
            f"scripted rear-four gait fell: {r}")
        assert r["lift_duty_tail"] < 0.15, (
            f"'lifted' fronts touch the ground {r['lift_duty_tail']:.2f} "
            "of the tail — tuck pose not clearing")


def test_quadwalk_gait_beats_sixleg_walk(quadwalk_bank):
    """Rear-four stepping with fronts up must OUT-EARN the six-leg walk
    under the quadwalk stack, else PPO will just keep walking on six
    (the fronts-stepping cheat). The separator is the clear income the
    grounded fronts forfeit; the exemptions only stop the honest form
    being CHARGED."""
    qg, six = _qw_mean(quadwalk_bank, "quadgait"), _qw_mean(
        quadwalk_bank, "sixleg")
    assert qg > six + 100.0, (
        f"six-leg walking rivals the commanded quad gait: "
        f"quadgait {qg:.0f} vs sixleg {six:.0f}")


def test_quadwalk_gait_beats_frontdrag(quadwalk_bank):
    """Fronts-down dragging (feet planted, scraping along) must earn
    clearly less than lifting them: it forfeits the clear income AND
    pays the drag charges the exemptions deliberately did NOT lift."""
    qg, fd = _qw_mean(quadwalk_bank, "quadgait"), _qw_mean(
        quadwalk_bank, "frontdrag")
    assert qg > fd + 100.0, (
        f"fronts-down drag rivals the honest quad gait: "
        f"quadgait {qg:.0f} vs frontdrag {fd:.0f}")


def test_quadwalk_gait_beats_freeze(quadwalk_bank):
    """A quad-stance park (fronts up, nobody stepping) collects the
    full clear+plant income — walking must still win via the velocity
    kernel/progress, and the park-duty charge (support legs only, by
    the exemption) must bite the freeze."""
    qg, fz = _qw_mean(quadwalk_bank, "quadgait"), _qw_mean(
        quadwalk_bank, "freeze")
    assert qg > fz + 100.0, (
        f"freezing at the quad stance rivals walking: "
        f"quadgait {qg:.0f} vs freeze {fz:.0f}")


def test_quadwalk_never_sampled_by_default():
    """Default config: p_quadwalk exists (harness forcing needs the
    attribute) but is 0.0 — the mode must never be drawn, so every
    legacy lineage's episode stream is unchanged."""
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv

    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=False,
        dr_scale=0.0, episode_seconds=15.0, seed=SEEDS[0],
        cfg=load_config())
    assert getattr(env._goal_gen, "p_quadwalk", None) == 0.0
    modes = set()
    for _ in range(300):
        modes.add(env._sample_goal().mode)
    env.close()
    assert "quadwalk" not in modes, (
        "quadwalk sampled at default p=0 — cdf branch broken")


# k_quad_still (08-13): the learned quad HOLD stance creeps ~0.33 m/15 s
# (cw-quad-turn1-r1 harness measurement) because nothing prices body
# translation in quad mode (hold_still_gate exempts quad by design).
# The term charges body planar speed above a floor while NO velocity is
# commanded. Bank: a quad-stance "creeper" (rear-four skate, fronts
# tucked, body sliding at ~0.02 m/s) vs the still quad stance.

QUAD_STILL_OVERRIDES = dict(QUADWALK_OVERRIDES)
QUAD_STILL_OVERRIDES.update({("reward", "k_quad_still"): 50.0})


def _quad_hold_rollout(policy: str, seed: int, overrides: dict) -> float:
    """Quad HOLD mode (p_quad=1): 'still' holds the tuck stance,
    'creep' translates the body through the hold command.

    'still' needs the probe-derived viable statics: mid feet splayed
    forward 0.06 m (feasibility-GO geometry — without it the tuck
    stance pitch-trips in <1 s) and the tuck blended in over 1.5 s.
    'creep' is the plain six-leg gait at 0.03 m/s — the only SCRIPTED
    policy that genuinely translates (every scripted quad-stance
    creep attempt slips in place; see probe_quad_crawl.py). The term
    under test charges measured BODY SPEED while a quad hold is
    commanded, independent of leg pattern, so this proxy exercises
    exactly the mechanism that prices the learned rear-four creep
    (0.022 m/s, cw-quad-turn1-r1)."""
    from tripod_gait import TripodGait

    cfg = load_config()
    for (sec, leaf), val in overrides.items():
        cfg.setdefault(sec, {})[leaf] = val
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=False,
        dr_scale=0.0, episode_seconds=15.0, seed=seed, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad", "walk", "quadwalk"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "quad" else 0.0)
    env.reset()
    assert env._goal_traj.mode == "quad"
    gait = TripodGait(vx=0.0, lift=0.025 if policy == "creep" else 0.0)
    gait.sync_plant_stance(*WALK_PLANT)
    if policy != "creep":
        _orig_target = gait._foot_target_in_body

        def _splayed(i, vx, vy, om):
            dx, dy, dz = _orig_target(i, vx, vy, om)
            if i in (1, 4):
                dx += 0.06
            return (dx, dy, dz)
        gait._foot_target_in_body = _splayed
    plant_rad = np.array([0.0, *WALK_PLANT] * 6) * DEG2RAD
    gait.reset_phase()
    tuck = np.array(QW_TUCK_RAD)

    total, step = 0.0, 0
    while True:
        t = step * env.dt
        gait.set_velocity(vx=0.03 if policy == "creep" else 0.0, vy=0.0)
        q = np.asarray(gait.desired_deg(t)) * DEG2RAD
        if policy != "creep":
            a = min(t / 1.5, 1.0)
            for leg in QW_LIFT:
                q[3 * leg:3 * leg + 3] = (
                    (1 - a) * plant_rad[3 * leg:3 * leg + 3]
                    + a * tuck)
        _obs, r, term, trunc, _info = env.step(q_rad_to_action(q))
        total += float(r)
        step += 1
        if term or trunc:
            break
    env.close()
    return total


def test_quad_still_default_off_bit_exact():
    """k_quad_still=0.0 (explicit) must equal the key being absent."""
    off = dict(QUADWALK_OVERRIDES)
    off[("reward", "k_quad_still")] = 0.0
    a = _quad_hold_rollout("creep", SEEDS[0], QUADWALK_OVERRIDES)
    b = _quad_hold_rollout("creep", SEEDS[0], off)
    assert a == b, f"k_quad_still=0 changed the reward path ({a} vs {b})"


def test_quad_still_charges_the_creep_not_the_stand(quad_still_returns=None):
    """With k_quad_still=50 the sliding quad stance pays real money
    (the measured creep is ~0.022 m/s; 50 * ~0.017 ~= 0.85/tick) while
    the genuinely still stance is charged ~nothing (floor)."""
    s = SEEDS[0]
    creep_cost = (_quad_hold_rollout("creep", s, QUADWALK_OVERRIDES)
                  - _quad_hold_rollout("creep", s, QUAD_STILL_OVERRIDES))
    still_cost = (_quad_hold_rollout("still", s, QUADWALK_OVERRIDES)
                  - _quad_hold_rollout("still", s, QUAD_STILL_OVERRIDES))
    assert creep_cost > 100.0, (
        f"k_quad_still=50 charged the creeping stance only "
        f"{creep_cost:.1f} over the episode — no bite")
    assert still_cost < 0.2 * creep_cost, (
        f"still stance pays {still_cost:.1f} vs creep {creep_cost:.1f} "
        "— the floor is not protecting genuine stillness")


# k_quad_lift_contact (08-13): cw-quadwalk1 (income 1.5/1.0) and
# cw-quadwalk2 (3x income) both landed the pre-registered fronts-down
# cheat — front tail contact duty moved 1.0 -> 0.62/0.32 but never
# under the 0.15 fronts_lifted bar, so pure income pricing is a CLOSED
# lever (quad/STATUS.md two-miss fork). The term under test charges
# the fraction of commanded lift legs in ground contact per tick after
# grace, making six-leg walking strictly unprofitable. Bank: the
# scripted six-leg walk (the cheat) vs the tucked-fronts quadgait
# (honest form; does not need to translate to prove the charge is ~0
# for lifted fronts).

QUAD_LIFTC_OVERRIDES = dict(QUADWALK_OVERRIDES)
QUAD_LIFTC_OVERRIDES.update({("reward", "k_quad_lift_contact"): 3.0})


def test_quad_lift_contact_default_off_bit_exact():
    """k_quad_lift_contact=0.0 (explicit) must equal the key absent."""
    off = dict(QUADWALK_OVERRIDES)
    off[("reward", "k_quad_lift_contact")] = 0.0
    a = _quadwalk_rollout("sixleg", SEEDS[0],
                          overrides=QUADWALK_OVERRIDES)["return"]
    b = _quadwalk_rollout("sixleg", SEEDS[0], overrides=off)["return"]
    assert a == b, (
        f"k_quad_lift_contact=0 changed the reward path ({a} vs {b})")


def test_quad_lift_contact_charges_sixleg_not_quadgait():
    """With k=3 the six-leg walk (fronts stepping on the ground) pays
    real money over the episode while the tucked-fronts gait is
    charged ~nothing — the charge separates exactly on the cheat
    axis, not on stepping itself."""
    s = SEEDS[0]
    six_cost = (_quadwalk_rollout("sixleg", s,
                                  overrides=QUADWALK_OVERRIDES)["return"]
                - _quadwalk_rollout("sixleg", s,
                                    overrides=QUAD_LIFTC_OVERRIDES)["return"])
    qg_cost = (_quadwalk_rollout("quadgait", s,
                                 overrides=QUADWALK_OVERRIDES)["return"]
               - _quadwalk_rollout("quadgait", s,
                                   overrides=QUAD_LIFTC_OVERRIDES)["return"])
    assert six_cost > 100.0, (
        f"k=3 charged the six-leg cheat only {six_cost:.1f} over the "
        "episode — no bite")
    assert qg_cost < 0.2 * six_cost, (
        f"tucked-fronts gait pays {qg_cost:.1f} vs sixleg "
        f"{six_cost:.1f} — the charge is hitting the honest form")


def test_quad_lift_contact_charges_frontdrag_like_sixleg():
    """Fronts-down dragging (planted, not stepping) is charged at
    least as hard as the stepping cheat: contact duty ~1.0."""
    s = SEEDS[0]
    fd_cost = (_quadwalk_rollout("frontdrag", s,
                                 overrides=QUADWALK_OVERRIDES)["return"]
               - _quadwalk_rollout("frontdrag", s,
                                   overrides=QUAD_LIFTC_OVERRIDES)["return"])
    assert fd_cost > 100.0, (
        f"k=3 charged the fronts-down drag only {fd_cost:.1f} — no bite")


# --------------------------------------------------------------------
# goal.quadwalk_start (08-13, quad track, after cw-quadwalk3): with the
# lift-contact charge verifiably live (env/reward_quad_lift_contact
# -1.54/tick ~ -575/ep) the policy STILL walked on six legs — income
# and charge pricing are both exhausted, the blocker is exploration
# from the six-foot plant spawn. "quad" spawns quadwalk episodes
# already in the fronts-tucked four-leg stance (env-side kind
# "quadstance": support legs at plant, lift legs at the feasibility
# tuck claw, +-2 deg jitter). Default "plant" must stay bit-exact.

def test_quadwalk_start_default_plant_bit_exact():
    """Key absent and key explicitly 'plant' are the same episode:
    identical start_at literal and identical scripted-rollout return
    (no rng draw may be added on the default path)."""
    env = _make_quadwalk_env(SEEDS[0])
    env.reset()
    assert getattr(env._goal_traj, "start_at", "?") == "plant"
    env.close()
    explicit = dict(QUADWALK_OVERRIDES)
    explicit[("goal", "quadwalk_start")] = "plant"
    a = _quadwalk_rollout("sixleg", SEEDS[0],
                          overrides=QUADWALK_OVERRIDES)["return"]
    b = _quadwalk_rollout("sixleg", SEEDS[0], overrides=explicit)["return"]
    assert a == b, (
        f"goal.quadwalk_start='plant' changed the episode ({a} vs {b})")


def test_quadwalk_start_invalid_rejected():
    ov = dict(QUADWALK_OVERRIDES)
    ov[("goal", "quadwalk_start")] = "sideways"
    env = _make_quadwalk_env(SEEDS[0], overrides=ov)
    with pytest.raises(ValueError):
        env.reset()
    env.close()


def test_quadwalk_start_quad_spawns_fronts_up():
    """'quad' spawn: episode begins in the four-leg stance — lift-leg
    joints at the tuck claw (not plant), lift feet OFF the ground and
    all four support feet ON it right after the spawn settles, and the
    stance survives a 2 s scripted hold without tipping. Runs using
    this spawn must widen the tilt envelope past the limp-settle sag
    transient (~15-17 deg nose-down onto the claws) — same contract as
    the getup 'any' starts; the test mirrors the launch cfg (25 deg =
    the deployment envelope)."""
    ov = dict(QUADWALK_OVERRIDES)
    ov[("goal", "quadwalk_start")] = "quad"
    ov[("safety", "max_roll_deg")] = 25.0
    ov[("safety", "max_pitch_deg")] = 25.0
    env = _make_quadwalk_env(SEEDS[0], overrides=ov)
    env.reset()
    traj = env._goal_traj
    assert getattr(traj, "start_at", "?") == "quadstance"
    tuck = np.array(QW_TUCK_RAD)
    # Hold the spawn stance for 2 s — same construction as the spawn
    # itself and the bank's surviving freeze stance: TripodGait plant
    # with mid feet splayed +0.06 m forward, lift legs at tuck.
    from tripod_gait import TripodGait
    gait = TripodGait()
    gait.sync_plant_stance(*WALK_PLANT)
    _orig = gait._foot_target_in_body

    def _splayed(i, vx, vy, om, _o=_orig):
        dx, dy, dz = _o(i, vx, vy, om)
        if i in (1, 4):
            dx += 0.06
        return (dx, dy, dz)
    gait._foot_target_in_body = _splayed
    gait.set_velocity(vx=0.0, vy=0.0)
    gait.reset_phase()
    q_hold = np.asarray(gait.desired_deg(0.0)) * DEG2RAD
    for leg in QW_LIFT:
        q_hold[3 * leg:3 * leg + 3] = tuck
    n_hold = int(round(2.0 / env.dt))
    n_settle = int(round(0.5 / env.dt))
    lift_contact = support_off = checked = 0
    term = trunc = False
    for step in range(n_hold):
        _obs, _r, term, trunc, _info = env.step(q_rad_to_action(q_hold))
        if step >= n_settle:
            checked += 1
            lift_contact += sum(
                1 for leg in QW_LIFT
                if float(env.data.sensordata[env._touch_adr[leg]]) > 0.5)
            support_off += sum(
                1 for leg in range(6) if leg not in QW_LIFT
                and float(env.data.sensordata[env._touch_adr[leg]]) < 0.5)
        if term or trunc:
            break
    env.close()
    assert not term, "quadstance spawn tipped over during a scripted hold"
    assert checked > 0
    lift_duty = lift_contact / (2.0 * checked)
    support_off_frac = support_off / (4.0 * checked)
    assert lift_duty < 0.05, (
        f"lift feet touch the ground {lift_duty:.2f} of post-settle "
        "ticks — spawn is not fronts-up")
    assert support_off_frac < 0.25, (
        f"support feet off the ground {support_off_frac:.2f} of ticks "
        "— spawn stance not planted")


# --------------------------------------------------------------------
# reward.walk_gait_gate (08-13, quad track, after cw-quadwalk1-5):
# additive pricing is measured-exhausted for BOTH quadwalk cheat
# families — quadwalk3 verifiably PAID the -575/ep lift-contact
# charge (~40% of return) and kept walking on six legs; quadwalk5's
# 6x k_park_duty reprice changed the mid-leg-park scoot NOT AT ALL
# (identical legs [1,4] sacrificed 12/12). The anchor gate never sees
# an air-parked leg (its fraction spans LOADED feet only), so the
# structural fix is a new income gate: velocity income (kernel +
# positive progress + quadwalk clear/plant) is multiplied by the MIN
# over commanded SUPPORT legs of a per-leg "completed a real swing
# recently" score (window 2 s of commanded ticks, linear 2 s fade,
# swing = >=2 ticks airborne + XY stride >= gait_gate_stride_mm).
# MIN, not mean: fractional discounts are measured-payable; parking
# ANY support leg must collapse transport income to the (1-g) floor.
# Lift legs are exempt (their stepping is the six-leg cheat, priced
# elsewhere); penalties never shrink; default 0 = off, bit-exact.
# NOTE the scripted splayed rear-four trot canNOT serve as the honest
# actor here: its mid legs are PINNED (0 completed swings after the
# blend — the probe_quad_crawl geometric finding), so the gate
# correctly scores IT as a sacrificing gait; the honest-form check
# lives in WALK mode, where a genuinely six-leg-cycling scripted
# reference exists.

GAIT_GATE_ON = dict(WALK_OVERRIDES)
GAIT_GATE_ON[("reward", "walk_gait_gate")] = 1.0
QW_GAIT_GATE_ON = dict(QUADWALK_OVERRIDES)
QW_GAIT_GATE_ON[("reward", "walk_gait_gate")] = 1.0
# Raised flag pose (hip up, knee tucked) for a sacrificed leg.
GG_FLAG_RAD = (0.0, -1.10, 2.40)


def _gait_gate_walk_rollout(policy: str, seed: int,
                            overrides: dict) -> dict:
    """Walk-mode rollout: 'gait' = the honest six-leg scripted tripod;
    'flagleg' = the same gait with mid leg 1 blended to a raised flag
    pose (the one-leg-sacrifice cheat class). Returns the episode
    return plus the summed gated income terms and the post-6s median
    of the walk_gait_min metric."""
    from tripod_gait import TripodGait

    env = _make_walk_env(seed, overrides)
    env.reset()
    traj = env._goal_traj
    n = len(traj.vx)
    hold_n = ramp_n = int(round(1.0 / env.dt))
    traj.vx[:] = WALK_CMD_VX
    traj.vx[:hold_n] = 0.0
    traj.vx[hold_n:hold_n + ramp_n] = WALK_CMD_VX * np.linspace(
        0.0, 1.0, ramp_n)
    traj.vy[:] = 0.0
    if traj.wz is not None:
        traj.wz[:] = 0.0
    gait = TripodGait(vx=0.0, lift=0.025)
    gait.sync_plant_stance(*WALK_PLANT)
    plant_rad = np.array([0.0, *WALK_PLANT] * 6) * DEG2RAD
    flag = np.array(GG_FLAG_RAD)
    gait.reset_phase()
    total, step = 0.0, 0
    kernel = prog = 0.0
    gmin_tail: list[float] = []
    while True:
        t = step * env.dt
        i = min(step, n - 1)
        gait.set_velocity(vx=float(traj.vx[i]), vy=float(traj.vy[i]))
        q = np.asarray(gait.desired_deg(t)) * DEG2RAD
        if policy == "flagleg":
            a = min(t / 1.5, 1.0)
            q[3:6] = (1 - a) * plant_rad[3:6] + a * flag
        _obs, r, term, trunc, info = env.step(q_rad_to_action(q))
        total += float(r)
        kernel += float(info.get("reward_walk", 0.0))
        prog += float(info.get("reward_walk_prog", 0.0))
        if "walk_gait_min" in info and t > 6.0:
            gmin_tail.append(float(info["walk_gait_min"]))
        step += 1
        if term or trunc:
            break
    env.close()
    return {"return": total, "terminated": bool(term), "kernel": kernel,
            "prog": prog,
            "gmin": (float(np.median(gmin_tail)) if gmin_tail else None)}


def _gait_gate_midpin_rollout(seed: int, overrides: dict) -> dict:
    """Quadwalk-mode rollout of the MID-LEG-PIN sacrifice: the splayed
    rear-four trot with the mid legs FROZEN at their splayed plant
    pose (drag anchors, duty ~1.0, zero completed swings) and fronts
    honestly tucked — the scripted twin of cw-quadwalk4/5's cheat
    family (statically stable, unlike the air-park scoot, so the
    15 s episode survives and the gate window is actually exercised)."""
    from tripod_gait import TripodGait

    env = _make_quadwalk_env(seed, overrides)
    env.reset()
    traj = env._goal_traj
    assert getattr(traj, "mode", "") == "quadwalk"
    n = len(traj.vx)
    hold_n = int(round(2.0 / env.dt))
    ramp_n = int(round(1.0 / env.dt))
    traj.vx[:] = 0.03
    traj.vx[:hold_n] = 0.0
    traj.vx[hold_n:hold_n + ramp_n] = 0.03 * np.linspace(
        0.0, 1.0, ramp_n)
    traj.vy[:] = 0.0
    if traj.wz is not None:
        traj.wz[:] = 0.0
    gait = TripodGait(vx=0.0, lift=0.025)
    gait.sync_plant_stance(*WALK_PLANT)
    _orig = gait._foot_target_in_body

    def _splayed(i, vx, vy, om, _o=_orig):
        dx, dy, dz = _o(i, vx, vy, om)
        if i in (1, 4):
            dx += 0.06
        return (dx, dy, dz)
    gait._foot_target_in_body = _splayed
    plant_rad = np.array([0.0, *WALK_PLANT] * 6) * DEG2RAD
    gait.reset_phase()
    tuck = np.array(QW_TUCK_RAD)
    gait.set_velocity(vx=0.0, vy=0.0)
    q_pin = np.asarray(gait.desired_deg(0.0)) * DEG2RAD
    total, step = 0.0, 0
    income = 0.0
    gmin_tail: list[float] = []
    while True:
        t = step * env.dt
        i = min(step, n - 1)
        gait.set_velocity(vx=float(traj.vx[i]), vy=float(traj.vy[i]))
        q = np.asarray(gait.desired_deg(t)) * DEG2RAD
        a = min(t / 1.5, 1.0)
        for leg in QW_LIFT:
            q[3 * leg:3 * leg + 3] = (
                (1 - a) * plant_rad[3 * leg:3 * leg + 3] + a * tuck)
        for leg in (1, 4):
            q[3 * leg:3 * leg + 3] = q_pin[3 * leg:3 * leg + 3]
        _obs, r, term, trunc, info = env.step(q_rad_to_action(q))
        total += float(r)
        for k in ("reward_walk", "reward_quad_clear",
                  "reward_quad_plant"):
            income += float(info.get(k, 0.0))
        income += max(float(info.get("reward_walk_prog", 0.0)), 0.0)
        if "walk_gait_min" in info and t > 10.0:
            gmin_tail.append(float(info["walk_gait_min"]))
        step += 1
        if term or trunc:
            break
    env.close()
    return {"return": total, "terminated": bool(term), "income": income,
            "gmin": (float(np.median(gmin_tail)) if gmin_tail else None)}


def test_walk_gait_gate_default_off_bit_exact():
    """walk_gait_gate=0.0 (explicit) must equal the key absent, on
    both a quadwalk and a walk rollout (no reward-path or rng
    change on the default path)."""
    off = dict(QUADWALK_OVERRIDES)
    off[("reward", "walk_gait_gate")] = 0.0
    a = _quadwalk_rollout("sixleg", SEEDS[0],
                          overrides=QUADWALK_OVERRIDES)["return"]
    b = _quadwalk_rollout("sixleg", SEEDS[0], overrides=off)["return"]
    assert a == b, (
        f"walk_gait_gate=0 changed the quadwalk reward path ({a} vs {b})")
    woff = dict(WALK_OVERRIDES)
    woff[("reward", "walk_gait_gate")] = 0.0
    c = _gait_gate_walk_rollout("gait", SEEDS[0], WALK_OVERRIDES)["return"]
    d = _gait_gate_walk_rollout("gait", SEEDS[0], woff)["return"]
    assert c == d, (
        f"walk_gait_gate=0 changed the walk reward path ({c} vs {d})")


def test_walk_gait_gate_keeps_honest_gait_income():
    """The honest six-leg scripted gait (every support leg completes
    real swings well inside the 2 s window — measured qualifying-swing
    gap median 0.76 s) must keep its income under the full gate:
    factor pinned at 1, return within a few percent of gate-off."""
    off = _gait_gate_walk_rollout("gait", SEEDS[0], WALK_OVERRIDES)
    on = _gait_gate_walk_rollout("gait", SEEDS[0], GAIT_GATE_ON)
    assert on["gmin"] is not None and on["gmin"] >= 0.99, (
        f"honest gait scored walk_gait_min {on['gmin']} — the gate is "
        "mis-scoring a genuinely cycling gait")
    assert not on["terminated"]
    drop = off["return"] - on["return"]
    assert drop < 0.05 * abs(off["return"]) + 20.0, (
        f"gate cost the honest gait {drop:.1f} of {off['return']:.1f} "
        "— it must be ~free for the intended behavior")


def test_walk_gait_gate_collapses_flag_leg_income():
    """One sacrificed leg (mid leg 1 raised to a flag) must collapse
    velocity income to the floor: walk_gait_min 0 after the fade and
    a large return hit vs gate-off (measured: kernel 167->74, prog
    29->-29, return 387->235 at seed 0). This is the generic
    'sacrifice any subset' close — the MIN makes 5 honest legs unable
    to fund the 6th's park."""
    off = _gait_gate_walk_rollout("flagleg", SEEDS[0], WALK_OVERRIDES)
    on = _gait_gate_walk_rollout("flagleg", SEEDS[0], GAIT_GATE_ON)
    assert on["gmin"] is not None and on["gmin"] <= 0.01, (
        f"flag-leg gait still scores walk_gait_min {on['gmin']} — the "
        "sacrificed leg is not collapsing the min")
    hit = off["return"] - on["return"]
    assert hit > 80.0, (
        f"gate charged the flag-leg cheat only {hit:.1f} over the "
        "episode — no structural bite")
    assert on["kernel"] < 0.55 * off["kernel"], (
        f"flag-leg kernel income {on['kernel']:.1f} vs {off['kernel']:.1f} "
        "gate-off — transport income not collapsed")


def test_walk_gait_gate_collapses_quadwalk_midpin_income():
    """cw-quadwalk4/5's cheat family, scripted: mids pinned as drag
    anchors while the rears cycle and the fronts stay honestly up.
    Under the full gate the positive income streams (kernel, positive
    progress, clear, plant) must collapse (measured: 934->260 at seed
    0, return 1194->550) and walk_gait_min must sit at 0 — the pinned
    legs never complete a swing, so the sitting income can no longer
    fund the scoot. Charges are untouched by construction."""
    off = _gait_gate_midpin_rollout(SEEDS[0], QUADWALK_OVERRIDES)
    on = _gait_gate_midpin_rollout(SEEDS[0], QW_GAIT_GATE_ON)
    assert not off["terminated"] and not on["terminated"], (
        "midpin actor tipped — the sacrifice actor must survive for "
        "the gate window to be exercised")
    assert on["gmin"] is not None and on["gmin"] <= 0.01, (
        f"pinned-mid gait still scores walk_gait_min {on['gmin']}")
    assert on["income"] < 0.40 * off["income"], (
        f"gated income {on['income']:.1f} vs {off['income']:.1f} "
        "gate-off — the mid-leg sacrifice still collects")
    hit = off["return"] - on["return"]
    assert hit > 300.0, (
        f"gate cost the mid-pin scoot only {hit:.1f} over the episode "
        "— quadwalk5's cheat would still pay")


# --------------------------------------------------------------------------
# FULLCIRCLE bank — the cw-mt-c2-fullcircle1 stack (operator directive
# fb_20260815T114414, 08-15): translation-only full-circle commands
# (heading uniform [-pi, pi], speed 0.03-0.06, NO stop segments,
# wz identically zero with the yaw obs channel kept for checkpoint
# width) + the all-support-leg walk_gait_gate + the NEW early-fall
# horizon cost reward.term_cost_per_remaining_s. The exploit this bank
# pins: cw-mt-c2 at 20M retained POSITIVE return (~166/ep) from ~6 s
# flag-leg drag-then-fall trajectories in 15 s episodes — the flat -10
# safety_termination_penalty made early death cheaper than honest
# survival. Required ordering under the arm's FULL stack, per
# direction: honest gait > stall > park, and drag-then-fall < 0 (and
# far below a full-session freeze).

FC_TERM_COST_PER_S = 12.0
FC_OVERRIDES = {
    ("reward", "k_drag_loaded"): 10.0,
    ("reward", "k_park_duty"): 1.0,
    ("reward", "walk_kernel_prog_gate"): 1.0,
    ("reward", "walk_anchor_gate"): 1.0,
    ("reward", "anchor_tol_mm"): 10.0,
    ("reward", "walk_gait_gate"): 1.0,
    ("reward", "term_cost_per_remaining_s"): FC_TERM_COST_PER_S,
    ("goal", "walk_obs_body_vel"): 2.0,
    ("goal", "walk_yaw_cmd"): 1.0,
    ("goal", "walk_speed_min_m_s"): 0.03,
    ("goal", "walk_speed_max_m_s"): 0.06,
    ("goal", "walk_heading_max_rad"): math.pi,
    ("goal", "walk_stop_frac"): 0.0,
    ("goal", "walk_cmd_metrics"): 1.0,
    ("safety", "max_roll_deg"): 25.0,
    ("safety", "max_pitch_deg"): 25.0,
}

FC_CMDS = {
    "forward": (WALK_CMD_VX, 0.0),
    "backward": (-WALK_CMD_VX, 0.0),
    "crab_left": (0.0, WALK_CMD_VX),
    "diag_back_right": (-WALK_CMD_VX * 0.707, -WALK_CMD_VX * 0.707),
}


def test_joymodes_direct_command_score_orders_exact_direction_first():
    """The direct joystick term must make refusal and wrong-direction
    motion strictly worse than matching the requested velocity."""
    from rl_move.sim.walk_task import walk_cmd_track_score

    speed = 0.05
    exact = walk_cmd_track_score(speed, 0.0, speed, 0.0)[0]
    parked = walk_cmd_track_score(0.0, 0.0, speed, 0.0)[0]
    cross = walk_cmd_track_score(0.0, speed, speed, 0.0)[0]
    wrong = walk_cmd_track_score(-speed, 0.0, speed, 0.0)[0]
    assert (exact, parked, cross, wrong) == pytest.approx(
        (1.0, -1.0, -2.0, -3.0))
    assert walk_cmd_track_score(0.0, 0.0, 0.0, 0.0)[0] == 0.0
    assert walk_cmd_track_score(speed, 0.0, 0.0, 0.0)[0] < 0.0


@pytest.fixture(scope="module")
def fullcircle_returns() -> dict[str, dict[str, float]]:
    return {name: {p: float(np.mean(
        [_walk_rollout(p, s, vx=vx, vy=vy, overrides=FC_OVERRIDES)
         for s in SEEDS]))
        for p in ("gait", "stall", "park")}
        for name, (vx, vy) in FC_CMDS.items()}


def test_fullcircle_gait_beats_stall_and_park_every_direction(
        fullcircle_returns):
    """The gait gate + horizon fall cost must not invert the basic
    ordering anywhere on the circle: the hardware-proven tripod gait
    (which survives all 15 s, so the fall cost never fires on it)
    out-earns march-in-place and refusal in every direction."""
    for name, r in fullcircle_returns.items():
        assert r["gait"] > r["stall"] + 50.0, (
            f"{name}: stall rivals the gait under the FC stack: {r}")
        assert r["gait"] > r["park"] + 50.0, (
            f"{name}: parking rivals the gait under the FC stack: {r}")


def test_fullcircle_directions_priced_comparably(fullcircle_returns):
    """No commanded direction may be structurally cheap/expensive
    under the FC stack (min gait income > 55% of max), else the run
    re-learns a heading bias instead of the command."""
    gains = {n: r["gait"] for n, r in fullcircle_returns.items()}
    lo, hi = min(gains.values()), max(gains.values())
    assert lo > 0.55 * hi, (
        f"direction income skew under FC stack: {gains}")


def test_fullcircle_drag_then_fall_cannot_retain_positive_return():
    """THE c2 exploit, priced: accumulate the zero-lift skate's real
    income for the first 6 s of a 60 s episode under the FC stack,
    then charge the exact termination cost the env would levy on a
    fall at that tick (flat penalty + k * remaining seconds). The
    total must be NEGATIVE (c2 banked ~+166 this way at the flat -10),
    and a full-session freeze (park, 60 s, never falls) must out-earn
    it by a wide margin — death is never a paying strategy, freezing
    beats dying, and (per the ordering tests above) walking beats
    freezing."""
    from tripod_gait import TripodGait

    fall_tick = None
    drag_income = 0.0
    env = _make_walk_env(0, overrides=FC_OVERRIDES, episode_seconds=60.0)
    env.reset()
    traj = env._goal_traj
    n = len(traj.vx)
    hold_n = ramp_n = int(round(1.0 / env.dt))
    ramp = np.linspace(0.0, 1.0, ramp_n)
    traj.vx[:] = WALK_CMD_VX
    traj.vx[:hold_n] = 0.0
    traj.vx[hold_n:hold_n + ramp_n] = WALK_CMD_VX * ramp
    traj.vy[:] = 0.0
    if traj.wz is not None:
        traj.wz[:] = 0.0
    gait = TripodGait(vx=0.0, lift=0.0)          # zero-lift skate/drag
    gait.sync_plant_stance(*WALK_PLANT)
    gait.reset_phase()
    six_s = int(round(6.0 / env.dt))
    for step in range(six_s):
        i = min(step, n - 1)
        gait.set_velocity(vx=float(traj.vx[i]), vy=0.0)
        act = q_rad_to_action(
            np.asarray(gait.desired_deg(step * env.dt)) * DEG2RAD)
        _o, r, term, trunc, _i = env.step(act)
        drag_income += float(r)
        if term or trunc:
            fall_tick = step + 1
            break
    if fall_tick is None:
        # Counterfactual fall at 6 s: charge exactly what the env's
        # termination sites would charge at this step index.
        fall_tick = six_s
        pen = 10.0 + FC_TERM_COST_PER_S * max(
            env.episode_steps - fall_tick, 0) * env.dt
        drag_income -= pen
    env.close()

    park_total = _walk_rollout("park", 0, overrides=FC_OVERRIDES)
    # park is 15 s; scale to the 60 s session for the comparison
    # (income is per-tick and the park never terminates).
    park_60s = park_total * 4.0
    assert drag_income < 0.0, (
        f"drag-then-fall at 6 s of a 60 s episode still retains "
        f"positive return ({drag_income:.1f}) — raise "
        f"term_cost_per_remaining_s (the c2 exploit still pays)")
    assert park_60s > drag_income + 200.0, (
        f"freezing ({park_60s:.1f}) does not clearly out-earn early "
        f"death ({drag_income:.1f})")


# ---------------------------------------------------------------------------
# RECOVER bank — recover_to_plant (08-15 operator directive
# fb_20260815T165306_606974): from any recoverable state, reach a
# full-height, level, quiet stand with ALL SIX feet loaded, hold it
# 0.5 s, episode ends on held success. Reward is a potential
# DIFFERENCE (PBRS) on bounded features + one-shot success bonus +
# rate-normalized time tax; no occupancy/hold income, no alive bonus.
# MDP_PREFLIGHT: every stand-campaign cheat is banked before the first
# training run.
#
#   freeze    stay at the spawn — must bleed the time tax + terminal
#             fail cost (no alive income anywhere).
#   flagleg   five legs rise, one flags — the SMOOTH-MIN feature M
#             must keep the unloaded foot visible (no mean-average
#             loophole; the getup3-c2/getup4 plateau class) and block
#             success outright.
#   stilt     hip 0 / knee 80 pop — blocked by the H overshoot fade +
#             footprint/spread gates.
#   thrash    random flailing — must under-earn everything.
#   early-abort: a non-success termination pays >= the maximum
#             remaining time tax, so dying/aborting never out-earns
#             trying (rec_fail_cost floor).

RECOVER_OVERRIDES = {
    # Falls are recoverable states: side/back/upside-down spawns are
    # part of the task. 185 > the 180-deg attitude bound = the tilt
    # trip is genuinely OFF (a fully inverted settle reads ~179.5 and
    # must not terminate); timeout + the current/impact channels are
    # the only ends besides held success. Bank under the run's own
    # envelope.
    ("safety", "max_roll_deg"): 185.0,
    ("safety", "max_pitch_deg"): 185.0,
}


def _make_recover_env(seed: int, *, start: str,
                      extra: dict | None = None):
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv

    cfg = load_config()
    for (sec, leaf), val in RECOVER_OVERRIDES.items():
        cfg.setdefault(sec, {})[leaf] = val
    for (sec, leaf), val in (extra or {}).items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=False,
        dr_scale=0.0, episode_seconds=16.0, seed=seed, cfg=cfg)
    gen = env._goal_gen
    for attr in [a for a in vars(gen) if a.startswith("p_")]:
        setattr(gen, attr, 0.0)
    gen.p_recover = 1.0
    env.force_recover_start = start
    return env


def _recover_rollout(policy: str, seed: int, *, start: str = "zero",
                     extra: dict | None = None) -> dict:
    env = _make_recover_env(seed, start=start, extra=extra)
    env.reset()
    assert getattr(env, "_is_recover", False), "recover mode did not arm"
    ref = np.load(ROOT / RISE_REF)
    q_ref = ref["q_rad"]
    n_ref = len(q_ref)
    q0 = env.data.qpos[env._qadr].copy()
    q_stilt = np.array([0.0, 0.0, 80.0] * 6) * DEG2RAD
    rng = np.random.default_rng(seed)
    tot, step, succ, terminated = 0.0, 0, False, False
    bonus_sum, fail_sum, height_sum = 0.0, 0.0, 0.0
    last: dict = {}
    while True:
        t = step * env.dt
        if policy == "replay":
            act = q_rad_to_action(q_ref[min(step, n_ref - 1)])
        elif policy == "flagleg":
            q = q_ref[min(step, n_ref - 1)].copy()
            q[0:3] = q0[0:3]
            act = q_rad_to_action(q)
        elif policy == "freeze":
            act = q_rad_to_action(q0)
        elif policy == "stilt":
            act = q_rad_to_action(q0 if t < 2.0 else q_stilt)
        else:  # thrash
            act = rng.uniform(-1.0, 1.0, size=18)
        _obs, r, term, trunc, info = env.step(act)
        tot += float(r)
        step += 1
        bonus_sum += float(info.get("reward_recover_bonus", 0.0))
        fail_sum += float(info.get("reward_recover_fail", 0.0))
        height_sum += float(info.get("reward_height", 0.0))
        if info.get("recover_success", 0.0) > 0.0:
            succ = True
        last = info
        if term or trunc:
            terminated = term
            break
    env.close()
    return {"ret": tot, "succ": succ, "steps": step, "last": last,
            "terminated": terminated, "bonus": bonus_sum,
            "fail": fail_sum, "height": height_sum,
            "dt": 16.0 / max(step, 1) if step else 0.0}


RECOVER_POLICIES = ("replay", "flagleg", "freeze", "stilt", "thrash")


@pytest.fixture(scope="module")
def recover_bank() -> dict[str, list[dict]]:
    return {p: [_recover_rollout(p, s) for s in SEEDS[:2]]
            for p in RECOVER_POLICIES}


def test_recover_replay_succeeds_and_terminates(recover_bank):
    """The reference belly->plant rise reaches the held success: bonus
    paid EXACTLY once, episode ends by termination (not timeout),
    total return positive. This is the current-height target
    reachability proof (the postlower mechanically-impossible-target
    lesson) and the success-hold semantics check in one."""
    for roll in recover_bank["replay"]:
        assert roll["succ"], "reference rise never reached success"
        assert roll["terminated"], "success did not terminate"
        assert roll["steps"] < 395, "success only at the horizon"
        assert roll["ret"] > 0.0, f"honest recovery not paid: {roll['ret']:.1f}"
        b = float(roll["bonus"])
        assert b > 0.0, "success bonus never paid"
        # one-shot: total bonus equals a single payment
        first = [r for r in recover_bank["replay"]][0]
        assert abs(b - first["bonus"]) < 1e-9


def test_recover_replay_dominates_all_cheats(recover_bank):
    rets = {p: float(np.mean([r["ret"] for r in rolls]))
            for p, rolls in recover_bank.items()}
    best_cheat = max(rets[p] for p in RECOVER_POLICIES if p != "replay")
    assert rets["replay"] > best_cheat + 20.0, (
        f"recover stack prefers a known cheat: {rets}")
    for p in ("flagleg", "freeze", "stilt", "thrash"):
        assert not any(r["succ"] for r in recover_bank[p]), (
            f"cheat '{p}' reached the success gate")


def test_recover_smooth_min_keeps_unloaded_foot_visible(recover_bank):
    """No mean-only loophole: the flag-leg's final state must show M
    (smooth min per-foot load) far below L (mean load) AND a zero
    min-load, and success must be blocked."""
    for roll in recover_bank["flagleg"]:
        last = roll["last"]
        assert last["recover_min_load"] < 0.1
        assert last["recover_M"] < 0.45
        assert last["recover_L"] > last["recover_M"] + 0.25, (
            "M does not separate one unloaded foot from the mean")


def test_recover_no_early_abort_advantage(recover_bank):
    """A non-success end pays >= the maximum remaining time tax
    (rec_fail_cost floor 1.25 * c_time * horizon), so aborting early
    never out-earns a slow honest recovery — and the terminal fail
    cost actually fires at the timeout."""
    for roll in recover_bank["freeze"]:
        assert roll["fail"] < 0.0, "timeout paid no fail cost"
        # default floor: 1.25 * c_time(=1.0/s) * 16 s
        assert -roll["fail"] >= 16.0, (
            f"fail cost {-roll['fail']:.1f} below the max remaining "
            "time tax (early abort could pay)")
        assert roll["ret"] < 0.0, "doing nothing retains positive return"


def test_recover_no_height_charge_on_honest_rise(recover_bank):
    """The spawn-anchored h_err channel is disabled on recover ticks
    (same guard as getup): an honest rise must not accumulate
    reward_height charges (measured -58/ep before the guard)."""
    for roll in recover_bank["replay"]:
        assert abs(roll["height"]) < 1e-9, (
            f"recover episode accumulated reward_height "
            f"{roll['height']:.2f} — the h_err guard regressed")


def test_recover_time_tax_prices_stalling(recover_bank):
    """No alive/hold income: between the potential plateau and the
    horizon a freezer's per-tick income is negative (time tax +
    regularizer leakage)."""
    for roll in recover_bank["freeze"]:
        # total ex terminal fail, per tick
        per_tick = (roll["ret"] - roll["fail"]) / roll["steps"]
        assert per_tick < 0.0, (
            f"freeze earns non-negative per-tick income {per_tick:.4f}")


def test_recover_flip_spawn_is_nonupright():
    """Family-4 'flip' spawns settle genuinely tipped (side/back/
    upside-down), the episode survives the spawn (falls are
    recoverable, not terminal under the widened envelope), and the
    consume-once pending quat is cleared."""
    import math as _math
    tilts = []
    for seed in SEEDS:
        env = _make_recover_env(seed, start="flip")
        env.reset()
        assert getattr(env, "_flip_spawn_pending", None) is None
        r, p = env._true_roll_pitch()
        tilts.append(max(abs(r), abs(p)) * 180.0 / _math.pi)
        _obs, _r, term, _trunc, _info = env.step(np.zeros(18))
        assert not term, "flip spawn terminated on tick 1"
        env.close()
    assert max(tilts) > 60.0, (
        f"no flip spawn landed tipped (tilts {tilts})")


@pytest.mark.parametrize("start,bucket", (
    ("plant_catch", 0),
    ("onefoot_micro", 1),
    ("onefoot_mid", 2),
    ("onefoot", 3),
    ("park", 4),
))
def test_recover_near_goal_plant_teacher_reaches_held_success(
        start, bucket):
    """Every near-goal rung is reachable by the nominal plant teacher."""
    for seed in SEEDS[:2]:
        env = _make_recover_env(seed, start=start)
        _obs, reset_info = env.reset()
        assert env._goal_traj.start_kind == start
        assert reset_info["goal_mode"] == "recover"
        plant = q_rad_to_action(env._plant_deg * DEG2RAD)
        success = False
        last = {}
        while True:
            _obs, _r, term, trunc, last = env.step(plant)
            success = success or last.get("recover_success", 0.0) > 0.0
            if term or trunc:
                break
        env.close()
        assert success, f"plant teacher cannot solve settled {start}"
        assert last.get("termination_reason") == "recover_success"
        assert last["recover_start_kind_id"] >= 0.0
        assert last["recover_start_bucket"] == float(bucket)
        assert last["recover_active_families"] == 1.0
        assert last["recover_frontier_bucket"] == 0.0
        assert last[f"recover_episode_bucket_{bucket}"] == 1.0
        assert last[f"recover_success_bucket_{bucket}"] == 1.0


def test_recover_near_goal_buckets_increase_settled_disturbance():
    """B0-B4 must expand monotonically away from the plant manifold."""
    kinds = ("plant_catch", "onefoot_micro", "onefoot_mid",
             "onefoot", "park")
    distances = []
    for kind in kinds:
        per_seed = []
        for seed in SEEDS[:2]:
            env = _make_recover_env(seed, start=kind)
            env.reset()
            per_seed.append(float(np.linalg.norm(
                env.data.qpos[env._qadr]
                - env._plant_deg * DEG2RAD)))
            env.close()
        distances.append(float(np.mean(per_seed)))
    assert all(a < b for a, b in zip(distances, distances[1:])), (
        f"recovery difficulty bins are not monotonic: {distances}")


def test_recover_coarse_cliffs_are_split_into_single_distribution_rungs():
    env = _make_recover_env(0, start="plant_catch")
    expected = (
        "plant_catch", "onefoot_micro", "onefoot_mid", "onefoot", "park",
        "crouch_shallow", "crouch_mid", "crouch_deep", "partial_high",
        "partial_mid", "partial_low", "zero", "tangle_mild", "tangle_mid",
        "tangle_deep", "tangle", "flip")
    assert tuple(family[0] for family in env.RECOVER_FAMILIES) == expected
    assert all(len(family) == 1 for family in env.RECOVER_FAMILIES[:15])
    assert env.RECOVER_FAMILIES[15] == ("tangle", "bank")
    assert env.RECOVER_FAMILIES[16] == ("flip",)
    env.close()


def test_recover_floor_rungs_remain_distinct_after_physics_settle():
    """The added labels must describe different settled reset banks."""
    kinds = ("park", "crouch_shallow", "crouch_mid", "crouch_deep",
             "partial_high", "partial_mid", "partial_low", "zero",
             "tangle_mild", "tangle_mid", "tangle_deep", "tangle")
    sig = {}
    for kind in kinds:
        rows = []
        for seed in SEEDS[:3]:
            env = _make_recover_env(seed, start=kind)
            env.reset()
            q_dist = float(np.linalg.norm(
                env.data.qpos[env._qadr]
                - env._plant_deg * DEG2RAD))
            rows.append((q_dist, env._rec_reset_height_mm,
                         env._rec_reset_pad_spread_mm))
            env.close()
        sig[kind] = np.mean(rows, axis=0)

    assert sig["park"][0] < sig["crouch_shallow"][0]
    assert sig["crouch_shallow"][1] > sig["crouch_mid"][1] + 8.0
    assert sig["crouch_mid"][1] > sig["crouch_deep"][1] + 8.0
    assert sig["crouch_deep"][1] > sig["partial_high"][1] + 10.0
    assert sig["partial_high"][1] > sig["partial_mid"][1] + 8.0
    assert sig["partial_low"][0] > sig["partial_mid"][0] + 0.2
    spreads = [sig[k][2] for k in (
        "zero", "tangle_mild", "tangle_mid", "tangle_deep", "tangle")]
    assert all(a + 10.0 < b for a, b in zip(spreads, spreads[1:])), sig


def test_recover_periodic_eval_is_forced_and_split_by_bucket():
    from rl_move.sim.train_ppo_sim import (
        _recover_bucket_stats, _recover_split_stats)

    env = _make_recover_env(7, start="onefoot")
    plant = q_rad_to_action(env._plant_deg * DEG2RAD)
    near_goal = ("plant_catch", "onefoot_micro", "onefoot_mid",
                 "onefoot", "park")
    split = _recover_split_stats(
        env, lambda _obs: plant, per_kind=1, kinds=near_goal)
    buckets = _recover_bucket_stats(split)
    env.close()
    assert set(split) == set(near_goal)
    assert {v["bucket"] for v in split.values()} == set(range(5))
    assert all(v["successes"] == 1 for v in split.values())
    assert all(v["episodes"] == 1 for v in split.values())
    assert set(buckets) == set(range(5))
    assert all(v["success"] == 1.0 for v in buckets.values())
    assert all(v["episodes"] == 1 for v in buckets.values())


def test_recover_spaced_replay_and_monotonic_unlock():
    """The frontier stays dominant without ever deleting old buckets."""
    env = _make_recover_env(0, start="zero")
    env.force_recover_start = None
    assert env._rec_active_n == 1
    assert env._rec_focus_bucket == 0
    assert set(env._sample_recover().start_kind
               for _ in range(100)) == {"plant_catch"}

    # Admission does not move before the deterministic gate passes.
    env._rec_stats["plant_catch"] = (15, 30)
    env._recover_update_admission()
    assert env._rec_active_n == 1
    env._rec_stats["plant_catch"] = (4, 4)
    env._recover_update_admission()
    assert env._rec_active_n == 2
    assert env._rec_focus_bucket == 1
    assert env._recover_active_kinds() == [
        "plant_catch", "onefoot_micro"]
    assert "bank" not in env._recover_active_kinds(), (
        "bank admitted without a bank file")

    # A failed frontier assay no longer relocks it or any learned starts.
    env._rec_stats["onefoot_micro"] = (0, 6)
    env._recover_update_admission()
    assert env._rec_active_n == 2
    assert env._rec_focus_bucket == 1
    assert env._rec_stats["plant_catch"] == (4, 4)
    assert env._rec_stats["onefoot_micro"] == (0, 6)

    # At a later frontier, bucket mass is 50% focus, 25% recent-three,
    # 15% weakest old bucket, 10% all remaining old buckets.
    env._rec_active_n = 6
    env._rec_focus_bucket = 5
    env._rec_stats = {
        "plant_catch": (8, 8),
        "onefoot_micro": (8, 8),
        "onefoot_mid": (4, 8),
        "onefoot": (7, 8),
        "park": (8, 8),
    }
    env._recover_refresh_weak_bucket()
    assert env._rec_weak_bucket == 2
    np.testing.assert_allclose(
        env._recover_bucket_weights(),
        [0.05, 0.05, 0.20, 0.075, 0.125, 0.50])
    env.close()


def test_recover_curriculum_moves_only_from_certification():
    """Noisy PPO outcomes are telemetry; deterministic cert owns admission."""
    env = _make_recover_env(
        0, start="plant_catch",
        extra={("goal", "recover_external_certification"): 1.0})
    assert env._rec_active_n == 1
    assert env._rec_stats == {}

    # Four deterministic successes cross the configured 0.8 fraction gate.
    result = env.apply_recover_certification(
        "plant_catch", [True, True, True, True])
    assert result["success_fraction"] == 1.0
    assert result["successes"] == 4
    assert result["episodes"] == 4
    assert result["active_before"] == 1
    assert result["active_after"] == 2

    # A failed assay keeps the frontier unlocked and replayable.
    result = env.apply_recover_certification(
        "onefoot_micro", [False] * 6)
    assert result["success_fraction"] < 0.2
    assert result["active_before"] == 2
    assert result["active_after"] == 2
    assert result["focus_after"] == 1
    assert env._rec_stats["plant_catch"] == (4, 4)
    assert env._rec_stats["onefoot_micro"] == (0, 6)

    # Recovery on that same level advances normally.
    result = env.apply_recover_certification(
        "onefoot_micro", [True] * 6)
    assert result["active_after"] == 3
    assert result["focus_after"] == 2
    env.close()


def test_recover_external_cert_ignores_stochastic_terminal_for_admission():
    env = _make_recover_env(
        0, start="plant_catch",
        extra={("goal", "recover_external_certification"): 1.0})
    env.reset()
    plant = q_rad_to_action(env._plant_deg * DEG2RAD)
    last = {}
    while True:
        _obs, _reward, term, trunc, last = env.step(plant)
        if term or trunc:
            break

    assert last["termination_reason"] == "recover_success"
    assert env._rec_stats == {}
    assert env._rec_rollout_stats["plant_catch"][1] == 1
    assert env._rec_active_n == 1
    env.close()


def test_recover_empty_interval_and_walk_isolation():
    """p_recover=0 (the default) is an EMPTY cdf interval: an
    identically-seeded walk-mix env samples the identical trajectory
    whether the attribute is its default 0.0 or deleted outright, and
    a non-recover episode's info never carries recover keys (the
    pricing branch is never entered)."""
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv

    def mk(seed, drop_attr):
        cfg = load_config()
        env = SimHexapodJointWalkEnv(
            params=SimServoParams.from_cfg(None), randomize=False,
            dr_scale=0.0, episode_seconds=6.0, seed=seed, cfg=cfg)
        if drop_attr:
            del env._goal_gen.p_recover
        return env

    for seed in SEEDS[:2]:
        a, b = mk(seed, False), mk(seed, True)
        a.reset(seed=seed)
        b.reset(seed=seed)
        ta, tb = a._goal_traj, b._goal_traj
        assert ta.mode == tb.mode
        assert np.array_equal(ta.height, tb.height)
        va, vb = getattr(ta, "vx", None), getattr(tb, "vx", None)
        assert (va is None) == (vb is None)
        if va is not None:
            assert np.array_equal(va, vb)
        assert not getattr(a, "_is_recover", False)
        _obs, _r, _term, _trunc, info = a.step(np.zeros(a.n_act))
        assert "recover_phi" not in info
        a.close()
        b.close()


def test_recover_bc_anchor_gated_by_state():
    """train.bc_anchor_recover=1: the state-aligned rise anchor fires
    on the mastered rise manifold (upright belly start: level, feet
    carrying load) and NEVER on a flipped robot (orientation/height/
    contact conditioning — not nearest-q alone). Default-off: without
    the key no recover tick emits a target."""
    extra_on = {("train", "bc_anchor_coef"): 1.0,
                ("train", "bc_anchor_recover"): 1.0,
                ("reward", "rise_ref_path"): RISE_REF}
    env = _make_recover_env(0, start="zero", extra=extra_on)
    env.reset()
    _obs, _r, _t, _tr, info = env.step(np.zeros(18))
    assert "bc_target" in info and info.get("bc_mode") == 6, (
        "anchor did not fire on the upright belly start")
    env.close()

    env = _make_recover_env(0, start="flip", extra=extra_on)
    env.reset()
    _obs, _r, _t, _tr, info = env.step(np.zeros(18))
    assert "bc_target" not in info, (
        "anchor fired on a flipped robot — the orientation gate leaks")
    env.close()

    env = _make_recover_env(0, start="zero")  # key off
    env.reset()
    _obs, _r, _t, _tr, info = env.step(np.zeros(18))
    assert "bc_target" not in info, "anchor on without its cfg key"
    env.close()


def test_recover_rsi_default_off_and_cert_purity():
    """goal.recover_rsi_frac: default off = the flat-belly zero spawn,
    bit-exact; a FORCED kind (the deterministic CERT/eval path) never
    carries the RSI flag even at frac=1.0 — certification stays pure
    by construction."""
    env = _make_recover_env(0, start="zero")           # key absent
    env.reset()
    assert not getattr(env._goal_traj, "recover_rsi", False), (
        "RSI flag set without its cfg key")
    z = float(env.data.xpos[env._chassis_bid, 2])
    assert z < 0.050, (
        f"default zero spawn settled at z={z:.3f} m — not belly-flat")
    _obs, _r, _t, _tr, info = env.step(np.zeros(18))
    assert info.get("recover_rsi_episode") == 0.0
    env.close()

    extra = {("goal", "recover_rsi_frac"): 1.0,
             ("reward", "rise_ref_path"): RISE_REF}
    env = _make_recover_env(1, start="zero", extra=extra)  # forced kind
    env.reset()
    assert not getattr(env._goal_traj, "recover_rsi", False), (
        "RSI fired on a FORCED kind — CERT/eval purity broken")
    env.close()


def test_recover_rsi_spawns_on_ref_path():
    """goal.recover_rsi_frac=1.0: naturally drawn zero episodes carry
    the flag and spawn on the belly->plant reference path — the settle
    reaches supported heights the pure belly-flat start never has."""
    extra = {("goal", "recover_rsi_frac"): 1.0,
             ("reward", "rise_ref_path"): RISE_REF}
    env = _make_recover_env(2, start="zero", extra=extra)
    env.force_recover_start = None
    env._rec_active_n = 12          # unlock B0..B11 (zero = frontier)
    zs, hits = [], 0
    for _ in range(40):
        env.reset()
        kind = getattr(env._goal_traj, "start_kind", "")
        if kind != "zero":
            assert not getattr(env._goal_traj, "recover_rsi", False), (
                f"RSI flag leaked onto kind {kind!r}")
            continue
        assert getattr(env._goal_traj, "recover_rsi", False), (
            "naturally drawn zero episode missing the RSI flag at "
            "frac=1.0")
        zs.append(float(env.data.xpos[env._chassis_bid, 2]))
        hits += 1
        if hits >= 8:
            break
    assert hits >= 3, f"zero drawn only {hits} times with B11 unlocked"
    assert max(zs) > 0.055, (
        f"RSI spawns never settled above the belly (max z "
        f"{max(zs):.3f} m) — waypoints are not reaching supported "
        "mid-rise states")
    env.close()


def test_recover_rsi_stats_stay_clean():
    """An RSI episode's outcome must never touch the rollout EMA/
    counters or the C-trainer self-cert stats under the zero kind —
    it logs under its own _rsi suffix."""
    extra = {("goal", "recover_rsi_frac"): 1.0,
             ("reward", "rise_ref_path"): RISE_REF}
    env = _make_recover_env(3, start="zero", extra=extra)
    env.reset()
    env._goal_traj.recover_rsi = True   # forced kind: flag manually
    act = q_rad_to_action(env.data.qpos[env._qadr].copy())
    info = {}
    for _ in range(env.episode_steps + 2):
        _obs, _r, term, trunc, info = env.step(act)
        if term or trunc:
            break
    assert term or trunc, "episode never ended"
    assert info.get("recover_rsi_episode") == 1.0
    assert ("recover_episode_zero_rsi" in info
            or "recover_success_zero_rsi" in info), (
        "RSI episode did not log under its own _rsi suffix")
    assert "zero" not in env._rec_rollout_counts, (
        "RSI episode polluted the rollout counters for kind 'zero'")
    assert "zero" not in env._rec_stats, (
        "RSI episode polluted the self-cert stats for kind 'zero'")
    env.close()


def _write_rsi_bank(tmp_path, n: int = 4) -> str:
    """A tiny synthetic on-path bank for the harvested-bank RSI tests
    below — small, safely-in-limits joint offsets, never real harvest
    output (that's harvest_recover_rsi_bank.py's job, exercised
    separately as an integration script, not unit-tested here)."""
    rng = np.random.default_rng(0)
    q = rng.uniform(-15.0, 15.0, size=(n, 18)) * DEG2RAD
    path = tmp_path / "rsi_bank.npz"
    np.savez(path, q_rad=q)
    return str(path)


def test_recover_rsi_bank_default_off_and_cert_purity(tmp_path):
    """goal.recover_rsi_bank_frac: default off = the tangle family's
    own spawn, bit-exact; a FORCED kind never carries the bank flag
    even at frac=1.0 (CERT/eval purity, same contract as the ref-path
    RSI above)."""
    bank_path = _write_rsi_bank(tmp_path)
    env = _make_recover_env(4, start="tangle")   # key absent
    env.reset()
    assert not getattr(env._goal_traj, "recover_rsi_bank", False), (
        "bank RSI flag set without its cfg key")
    env.close()

    extra = {("goal", "recover_rsi_bank_frac"): 1.0,
             ("goal", "recover_rsi_bank_kinds"): "tangle",
             ("goal", "recover_rsi_bank_path"): bank_path}
    env = _make_recover_env(5, start="tangle", extra=extra)  # forced
    env.reset()
    assert not getattr(env._goal_traj, "recover_rsi_bank", False), (
        "bank RSI fired on a FORCED kind — CERT/eval purity broken")
    env.close()


def test_recover_rsi_bank_spawns_from_harvested_poses(tmp_path):
    """goal.recover_rsi_bank_frac=1.0: naturally drawn tangle episodes
    carry the bank flag (and only those, never a different kind); the
    actual spawn pose is read back post-settle (same limp-sag
    choreography as every other spawn path), so this checks the flag
    plumbing, not an exact pre-settle pose match (see the ref-path
    RSI test above for the same pattern)."""
    bank_path = _write_rsi_bank(tmp_path)
    extra = {("goal", "recover_rsi_bank_frac"): 1.0,
             ("goal", "recover_rsi_bank_kinds"): "tangle",
             ("goal", "recover_rsi_bank_path"): bank_path}
    env = _make_recover_env(6, start="tangle", extra=extra)
    env.force_recover_start = None
    env._rec_active_n = 16          # unlock B0..B15 (tangle = bucket 15)
    hits = 0
    for _ in range(60):
        env.reset()
        kind = getattr(env._goal_traj, "start_kind", "")
        if kind != "tangle":
            assert not getattr(env._goal_traj, "recover_rsi_bank",
                               False), (
                f"bank RSI flag leaked onto kind {kind!r}")
            continue
        assert getattr(env._goal_traj, "recover_rsi_bank", False), (
            "naturally drawn tangle episode missing the bank RSI flag "
            "at frac=1.0")
        assert env._tipped_applied, (
            "bank RSI spawn did not go through the waypoint-placement "
            "branch (_tipped_applied unset) — check sim_env dispatch")
        hits += 1
        if hits >= 5:
            break
    assert hits >= 2, f"tangle drawn only {hits} times with B15 unlocked"
    env.close()


def test_recover_rsi_bank_stats_stay_clean(tmp_path):
    """A bank-RSI episode's outcome must never touch the rollout EMA/
    counters or the C-trainer self-cert stats under the tangle kind —
    it logs under its own _rsibank suffix, distinct from the ref-path
    _rsi suffix."""
    bank_path = _write_rsi_bank(tmp_path)
    extra = {("goal", "recover_rsi_bank_frac"): 1.0,
             ("goal", "recover_rsi_bank_kinds"): "tangle",
             ("goal", "recover_rsi_bank_path"): bank_path}
    env = _make_recover_env(7, start="tangle", extra=extra)
    env.reset()
    env._goal_traj.recover_rsi_bank = True   # forced kind: flag manually
    act = q_rad_to_action(env.data.qpos[env._qadr].copy())
    info = {}
    for _ in range(env.episode_steps + 2):
        _obs, _r, term, trunc, info = env.step(act)
        if term or trunc:
            break
    assert term or trunc, "episode never ended"
    assert info.get("recover_rsi_bank_episode") == 1.0
    assert info.get("recover_rsi_episode") == 0.0, (
        "bank RSI episode also set the ref-path RSI flag")
    assert ("recover_episode_tangle_rsibank" in info
            or "recover_success_tangle_rsibank" in info), (
        "bank RSI episode did not log under its own _rsibank suffix")
    assert "tangle" not in env._rec_rollout_counts, (
        "bank RSI episode polluted the rollout counters for 'tangle'")
    assert "tangle" not in env._rec_stats, (
        "bank RSI episode polluted the self-cert stats for 'tangle'")
    env.close()
