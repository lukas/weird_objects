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


def _make_walk_env(seed: int, overrides: dict | None = None):
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv

    cfg = load_config()
    for (sec, leaf), val in (overrides or WALK_OVERRIDES).items():
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
    """'quiet' and 'hover' with per-foot contact-duty telemetry.

    hover     the crouchrise cheat class: two legs held a few mm off
              the ground — a FROZEN pose (stillness can't charge it)
              with all feet inside foot_down_mm (the clearance count
              can't charge it) and nothing near the flag band (no-flag
              can't charge it). Only measured load can.
    """
    env = _make_hold_env(seed, overrides)
    env.reset()
    q0 = env.data.qpos[env._qadr].copy()
    q = q0.copy()
    if policy == "hover":
        for leg in HOVER_LEGS:
            q[3 * leg + 1] -= HOVER_LIFT_DEG * DEG2RAD
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
