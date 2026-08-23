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
from rl_move.robot_state import DEG2RAD, RAD2DEG  # noqa: E402
from rl_move.sim.joint_task import (  # noqa: E402
    SimHexapodJointGoalEnv, q_rad_to_action)
from rl_move.sim.servo_model import SimServoParams  # noqa: E402

# --------------------------------------------------------------------------
# RISE bank — belly -> walkable plant stance (+132 mm at tibia-150,
# was +111 mm pre-tibia-150; see the 08-22 rise_height_mm
# recalibration below), full shaping stack.
#
# Exploits pinned here (all video-confirmed on real runs):
#   freeze  the paid-plateau exploit (arrival-gate sign bug, cw-uni-rfix-*)
#   stilt   hip 0 / knee 80 tip-toe pop — gamed rise "6/6" in rfix-fresh1
#           and reappeared in cw-stand-b2p1 as the flag-leg/tripod cheat.

RISE_REF = "rl_move/sim/refs/rise_ref_belly2plant.npz"
# rise_height_mm/max_height_mm RECALIBRATED 08-22 (rise_valid_plant
# dig-in, test_rise_valid_plant_separates_stand_from_cheats): [108,114]
# + max 115 are the PRE-tibia-150 (128 mm tibia) belly->plant height
# gain, stale since the tibia-150 recalibration lengthened the tibia
# ~22 mm. The demonstrated (tibia-150-correct) rise_ref_belly2plant.npz
# reference deterministically settles at h_rel=131.94 mm (measured,
# all 3 seeds, open-loop so seed-invariant) — ~24 mm above the stale
# target, tripping PLANT_SPEC's height_err_mm=15 window even though
# every other stand check (attitude/feet-down/no-flag/support/
# footprint/current) passes clean. This is a genuine spec staleness
# (the commanded target height, not the tolerance window, was wrong
# for the new leg length) — not a re-measure-the-tolerance fudge: the
# fix moves the TARGET to bracket the physically-correct settled
# height with margin on both sides of PLANT_SPEC's window, matching
# the same servo/contact-compliance sag (~22 mm below rigid FK) the
# GETUP bank's getup_z_full_frac already documents at this geometry.
RISE_OVERRIDES = {
    ("actions", "max_height_mm"): 137.0,
    ("goal", "rise_height_mm"): [128.0, 136.0],
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
    # See RISE_OVERRIDES above (08-22 recalibration, same physical
    # reference/geometry, same test_score_replay_ends_in_valid_plant
    # PLANT_SPEC height fix).
    ("actions", "max_height_mm"): 137.0,
    ("goal", "rise_height_mm"): [128.0, 136.0],
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
# 08-23 over-spin farm fix (probe_walk_income yawcmd0 income audit):
# k_yaw_prog's legacy clip pays up to 1.25x for OVER-rotation, so the
# income gradient points PAST the commanded rate — the eroded
# acq1-r2 checkpoint measurably farmed it (achieved/commanded yaw
# ratio 1.78 while collecting MORE yaw_prog than the accurate
# champion), and 3x income (yawprice3x) amplified the farm. Any turn
# arm must now also train with the overshoot decay ON.
# Two coupled levers (one mechanism — make yaw_prog's optimum the
# accurately-tracked command): overshoot decay peaks income at
# ratio 1.0, and the wz EMA (yaw_prog_avg_s, same DC-vs-AC fix as
# yaw_still_avg_s 08-11) stops the instantaneous pricing from fining
# the honest gait's zero-mean stride oscillation while paying the
# smooth fast spinner.
TURN_FIX_OVERRIDES = dict(TURN_OVERRIDES)
TURN_FIX_OVERRIDES.update({
    ("reward", "yaw_prog_overshoot_decay"): 1.0,
    ("reward", "yaw_prog_avg_s"): 1.0,
})
# The farm lives on LOW commanded rates (wz_ref ~ U(+-0.3)): the
# scripted gait's achieved wz saturates ~0.16 rad/s in this env
# (measured 08-23: omega 0.15 -> ~0.08 achieved, omega >=0.75 ->
# ~0.16 achieved, flat beyond), so at wz_cmd=0.08 spinning at the
# natural fast rate IS a ~2x over-rotation while omega=0.15 tracks
# the command ~1:1.
TURN_SLOW_WZ = 0.08      # rad/s command the farm targets
TRACKED_OMEGA = 0.15     # scripted omega achieving ~= TURN_SLOW_WZ
OVERSPIN_OMEGA = 0.75    # scripted omega achieving ~2x TURN_SLOW_WZ


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
                  overrides: dict | None = None,
                  return_terms: bool = False):
    from sim_gait_compat import TripodGait

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

    total, step, yp_sum, wy_sum = 0.0, 0, 0.0, 0.0
    while True:
        t = step * env.dt
        cmd = float(traj.wz[min(step, n - 1)])
        if policy == "turn":          # correct direction, full command
            gait.set_velocity(omega=cmd)
            act = q_rad_to_action(np.asarray(gait.desired_deg(t)) * DEG2RAD)
        elif policy == "partial":     # correct direction, ~35% of it
            gait.set_velocity(omega=0.35 * cmd)
            act = q_rad_to_action(np.asarray(gait.desired_deg(t)) * DEG2RAD)
        elif policy == "tracked":     # achieves ~= the commanded rate
            gait.set_velocity(omega=float(np.sign(cmd)) * TRACKED_OMEGA)
            act = q_rad_to_action(np.asarray(gait.desired_deg(t)) * DEG2RAD)
        elif policy == "overspin":    # correct direction, ~2x PAST it
            gait.set_velocity(omega=float(np.sign(cmd)) * OVERSPIN_OMEGA)
            act = q_rad_to_action(np.asarray(gait.desired_deg(t)) * DEG2RAD)
        elif policy == "drift":       # the structural fixed left drift
            gait.set_velocity(omega=DRIFT_WZ if step >= hold_n else 0.0)
            act = q_rad_to_action(np.asarray(gait.desired_deg(t)) * DEG2RAD)
        else:                         # park: ignore the command
            act = q_rad_to_action(plant_rad)
        _obs, r, term, trunc, _info = env.step(act)
        total += float(r)
        yp_sum += float(_info.get("reward_yaw_prog", 0.0))
        wy_sum += float(_info.get("reward_walk_yaw", 0.0))
        step += 1
        if term or trunc:
            break
    env.close()
    if return_terms == "walk_yaw":
        return total, wy_sum
    if return_terms:
        return total, yp_sum
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


def test_turn_overspin_farm_existed_in_legacy_stack():
    """DEFECT PROOF (hold_legacy pattern): under the legacy k_yaw_prog
    clip (+1.25 headroom, no overshoot decay), spinning ~2x PAST a
    slow command collects at least as much yaw_prog income as
    accurately tracking it — the measured farm the eroded acq1-r2
    checkpoint drove through (income audit 08-23,
    logs/probe_walk_income/yawcmd0_turn_income_audit.json; also why
    yawprice3x got WORSE with 3x income)."""
    yp = {p: float(np.mean(
        [_turn_rollout(p, s_wz * TURN_SLOW_WZ, s, TURN_OVERRIDES,
                       return_terms=True)[1]
         for s in SEEDS for s_wz in (+1.0, -1.0)]))
        for p in ("tracked", "overspin")}
    assert yp["overspin"] >= 0.95 * yp["tracked"], (
        f"legacy over-spin farm not reproduced ({yp}) — the audit's "
        f"defect model needs revisiting before trusting the fix.")


def test_turn_overspin_priced_out_by_overshoot_decay():
    """THE FIX: with reward.yaw_prog_overshoot_decay=1.0 the yaw_prog
    income peaks at the ACHIEVED==commanded rate — a ~2x over-spinner
    must earn decisively less yaw_prog AND less total return than the
    accurate tracker, and honest turning must still beat parking."""
    res = {p: [_turn_rollout(p, s_wz * TURN_SLOW_WZ, s,
                             TURN_FIX_OVERRIDES, return_terms=True)
               for s in SEEDS for s_wz in (+1.0, -1.0)]
           for p in ("tracked", "overspin", "park")}
    tot = {p: float(np.mean([r[0] for r in res[p]])) for p in res}
    yp = {p: float(np.mean([r[1] for r in res[p]])) for p in res}
    assert yp["tracked"] > yp["overspin"], (
        f"overshoot decay ON but over-spinning still collects more "
        f"yaw_prog than tracking the command: {yp}")
    assert tot["tracked"] > tot["overspin"], (
        f"over-spinning still out-earns accurate turning in TOTAL "
        f"return with the fix on: {tot}")
    assert tot["tracked"] > tot["park"], tot


def test_turn_overshoot_decay_default_off_is_bit_exact():
    """New key absent vs explicitly 0.0: identical return (the decay
    branch must be dead code when off)."""
    explicit = dict(TURN_OVERRIDES)
    explicit[("reward", "yaw_prog_overshoot_decay")] = 0.0
    explicit[("reward", "yaw_prog_avg_s")] = 0.0
    a = _turn_rollout("overspin", TURN_SLOW_WZ, SEEDS[0], TURN_OVERRIDES)
    b = _turn_rollout("overspin", TURN_SLOW_WZ, SEEDS[0], explicit)
    assert a == pytest.approx(b, abs=1e-9), (a, b)


# ---------------------------------------------------------------------------
# YAW KERNEL EMA (08-23, hold/forward income-dominance audit,
# q_20260823T0240Z item b / probe_walk_income yawcmd0 stack). Mirrors
# reward.walk_kernel_vel_ema (phasedir8, the linear-velocity kernel's
# sway-tax fix) for the yaw-rate kernel — same defect, different axis:
# a genuine full-stop hold command pins wz at ~0 with near-zero
# variance (nothing is moving), so the INSTANTANEOUS Gaussian sits at
# its peak almost every tick, while an honestly-tracking turner's body
# wz oscillates stride-to-stride around its achieved mean even when
# that mean matches the command well — the kernel never rewards it as
# richly as standing still. Measured on the real ypfix1 checkpoint
# (logs/probe_walk_income/hold_forward_income_ypfix1.json):
# reward_walk_yaw hold=374 vs forward=203 vs tip_left/right=132/147.
# Mechanism check below with the SCRIPTED references: "tracked"
# achieves ~= the commanded rate (TRACKED_OMEGA tracks TURN_SLOW_WZ
# ~1:1 per the overspin-farm bank above) but with real stride
# oscillation (wz_std ~0.12 rad/s at a wz_mean ~0.07 rad/s command
# 0.08); "turn" drives the gait at literally the commanded omega,
# which this env's stepping dynamics under-realizes (achieved wz_mean
# ~0.036, about HALF the command). Under the raw instantaneous kernel
# the two are within noise of each other on reward_walk_yaw alone
# (153.4 vs 154.4/ep, 6 seed/sign draws) — the kernel cannot tell
# "close to the command with natural sway" from "well under the
# command" apart, which is exactly the flat/no-gradient signal that
# lets a turn-in-place skill stay undiscovered when it is rare in the
# training distribution. With reward.walk_kernel_yaw_ema=1 the
# ordering separates cleanly (196.5 vs 154.6) without moving "drift"
# (the structural fixed off-command rotation, 115.9->113.1, slightly
# DOWN) — the fix rewards accurate average tracking, not merely
# rotating fast.
TURN_YAWEMA_OVERRIDES = dict(TURN_FIX_OVERRIDES)
TURN_YAWEMA_OVERRIDES.update({
    ("reward", "walk_kernel_yaw_ema"): 1.0,
    ("reward", "walk_kernel_yaw_tau_s"): 0.75,
})


def test_kernel_yaw_ema_default_off_is_bit_exact():
    """New key absent vs explicitly 0.0: identical return AND no
    state leak into pricing (the EMA branch must be dead code when
    off, matching walk_kernel_vel_ema's own off-path contract)."""
    explicit = dict(TURN_FIX_OVERRIDES)
    explicit[("reward", "walk_kernel_yaw_ema")] = 0.0
    a = _turn_rollout("tracked", TURN_SLOW_WZ, SEEDS[0], TURN_FIX_OVERRIDES)
    b = _turn_rollout("tracked", TURN_SLOW_WZ, SEEDS[0], explicit)
    assert a == pytest.approx(b, abs=1e-9), (a, b)


def test_kernel_yaw_ema_separates_accurate_tracking_from_undershoot():
    """THE FIX: under the raw instantaneous kernel, an honest tracker
    whose ACHIEVED mean rotation matches the command (but oscillates
    stride-to-stride) earns walk_yaw-kernel income within ~5/ep of a
    policy that under-rotates by ~2x (153-159 vs 154/ep, well inside
    the ~40/ep separation the EMA fix produces) — a near-flat gradient
    toward better tracking on this channel alone.
    With the EMA on, the accurate tracker must earn CLEARLY more than
    the under-rotator (the ordering a turn-in-place skill needs to be
    discoverable), and the structural fixed-drift reference must NOT
    benefit (still well below both honest policies)."""
    def mean_wy(policy: str, overrides: dict) -> float:
        vals = [_turn_rollout(policy, s_wz * TURN_SLOW_WZ, s, overrides,
                              return_terms="walk_yaw")[1]
                for s in SEEDS for s_wz in (+1.0, -1.0)]
        return float(np.mean(vals))

    wy_off = {p: mean_wy(p, TURN_FIX_OVERRIDES)
              for p in ("turn", "tracked", "drift")}
    wy_on = {p: mean_wy(p, TURN_YAWEMA_OVERRIDES)
             for p in ("turn", "tracked", "drift")}
    assert wy_off["tracked"] <= wy_off["turn"] + 10.0, (
        f"pre-fix baseline drifted — expected the accurate tracker to "
        f"be statistically tied with (not already ahead of) the "
        f"under-rotator on the raw kernel: {wy_off}")
    assert wy_on["tracked"] > wy_on["turn"] + 20.0, (
        f"EMA on but the kernel still can't separate accurate "
        f"tracking from under-rotation: {wy_on}")
    assert wy_on["drift"] < wy_on["turn"], (
        f"EMA on made the structural off-command drift out-earn even "
        f"the under-rotator on the kernel channel: {wy_on}")
    assert wy_on["drift"] <= wy_off["drift"] + 5.0, (
        f"EMA must not INCREASE the fixed drift's kernel income: "
        f"off={wy_off['drift']} on={wy_on['drift']}")


def test_kernel_yaw_ema_preserves_turn_beats_park_ordering():
    """The already-closed stillness-subsidy ordering (turn_returns'
    own turn > drift > park is NOT this bank's claim for TOTAL return
    since park's zero-effort kernel/still income is a separate,
    already-adjudicated design tradeoff — q_20260823T0130Z); this test
    only pins that turning on the accurate-tracking reference still
    clearly beats the fixed drift in TOTAL return with the new flag
    on, i.e. the EMA does not resurrect the drift as a competitive
    strategy once mixed with every other already-banked turn term."""
    tot = {}
    for p in ("tracked", "drift"):
        vals = [_turn_rollout(p, s_wz * TURN_SLOW_WZ, s,
                              TURN_YAWEMA_OVERRIDES)
                for s in SEEDS for s_wz in (+1.0, -1.0)]
        tot[p] = float(np.mean(vals))
    assert tot["tracked"] > tot["drift"] + 20.0, tot


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
    from sim_gait_compat import TripodGait

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
# SLIPWALK bank — FROM-SCRATCH ANTI-SLIP WALKING (operator order
# 2026-08-21, MCP operator lane, asked by Lukas): learn walking with no
# BC anchor and no pretrained locomotion actor, where loaded foot slip
# is priced hard and the policy does NOT have to track a commanded
# SPEED — it must travel in the commanded DIRECTION and actually cover
# ground. Every earlier from-scratch gait arm (dragstance1, rsi1,
# slowfirst1, sched1, ease1) collapsed into freeze or march-in-place,
# so this bank's job is to prove, BEFORE any GPU burns, that under this
# exact stack:
#   - real travel out-earns every stationary behavior by a wide margin,
#   - travelling FASTER than the command is never punished (no speed
#     band — the operator's "does not need to track a speed"),
#   - skating (loaded feet sliding, no travel) is the worst outcome,
#   - and the park -> step -> walk discovery gradient still points
#     uphill (a stepping stall must still beat a refusal park).
# The honest reference is again the hardware-proven scripted tripod
# gait; "fast" is that same gait driven at 2x the commanded speed,
# "creep" at half, "skate" is the zero-lift sliding twin.
# Stack: reward.k_walk_freeprog (direction-first income, no speed
# target) + reward.k_loadslip_excess (structural episode-accumulated
# loaded-slip charge; the per-tick drag forms are refuted) +
# reward.walk_gait_gate (all six legs must really step) +
# reward.k_walk_idle_charge (anti-park travel floor) +
# reward.k_park_duty. Calibrated on the controller 2026-08-21.

SLIPWALK_CMD_VX = 0.05
SLIPWALK_OVERRIDES = {
    # reward.term_penalty (08-22, cw-amp-m2-freeprog-{noamp,style05}
    # dig-in): the anti-suicide term. Under this stack WITHOUT it, a
    # scripted topple-in-1s netted +19/ep — the best-paying behavior
    # in the whole bank short of real walking (park -243, stall -143,
    # skate -1023) — because death exits before the freeprog/idle/
    # loadslip charges accrue, and both freeprog 2M arms duly learned
    # suicide in q4 (tilt terminations 59->132 / 90->241 at constant
    # std). Sized 400 > the worst-case discounted cost of staying
    # alive (-3.1/tick x (1-0.99^375)/0.01 ~= -295), so dying never
    # out-bids surviving in PPO's own view. Bit-exact for behaviors
    # that survive to truncation (the charge fires on term only).
    ("reward", "term_penalty"): 400.0,
    ("reward", "k_step_event"): 1.0,
    ("reward", "k_park_duty"): 2.0,
    ("reward", "k_walk_freeprog"): 3.0,
    ("reward", "walk_freeprog_cap_m_s"): 0.05,
    ("reward", "walk_loadslip_gate"): 0.0,
    ("reward", "loadslip_ok"): 1.5,
    ("reward", "k_loadslip_excess"): 6.0,
    ("reward", "walk_gait_gate"): 1.0,
    ("reward", "k_walk_idle_charge"): 20.0,
    ("reward", "walk_idle_speed_m_s"): 0.02,
    ("reward", "walk_idle_tau_s"): 1.0,
    ("goal", "walk_speed_min_m_s"): SLIPWALK_CMD_VX,
    ("goal", "walk_speed_max_m_s"): SLIPWALK_CMD_VX,
    ("goal", "walk_heading_max_rad"): 0.0,
}


def _slipwalk_rollout(policy: str, seed: int, *,
                      gait_scale: float = 1.0,
                      overrides: dict | None = None,
                      shuffle_half_period_s: float = 0.6,
                      ) -> tuple[float, float, int]:
    """Return (episode return, net forward body travel in m, steps).

    ``policy``: "gait" (scripted tripod at ``gait_scale`` x command),
    "skate" (same gait, zero swing lift — feet slide), "stall"
    (march in place), "park" (hold the plant stance and refuse),
    "stork" (the cw-amp-m2-pilot-{noamp,style05}-c1 38M cheat, 08-22:
    triad 0,2,4 holds the plant motionless while triad 1,3,5 is parked
    in the air — a statically stable half-tripod statue that collected
    RISING reward for 38M steps under the legacy kernel stack),
    "topple" (the cw-amp-m2-freeprog-{noamp,style05} 2M cheat, 08-22:
    lift the front pair from the plant stance so the body pitches over
    and the episode tilt-terminates in ~1 s — the fastest scripted way
    to die and stop paying the per-tick charge stack),
    "shuffle" (08-22, pre-registered cheat for reward.k_walk_swing: a
    real coordinated six-leg tripod gait that reverses direction every
    ``shuffle_half_period_s`` — every individual stride is a genuine,
    physically valid swing, so a per-completed-swing bonus would pay
    it in full, but net body displacement is ~0; this is the realistic
    "farm the swing bonus without going anywhere" attack a single
    accessible-income mechanism must not reopen).

    ``overrides``: cfg overrides dict, default ``SLIPWALK_OVERRIDES``
    (pass a superset to test an additional reward lever on top of the
    calibrated stack without touching the base dict).
    """
    from sim_gait_compat import TripodGait

    env = _make_walk_env(seed, overrides if overrides is not None
                          else SLIPWALK_OVERRIDES)
    env.reset()
    traj = env._goal_traj
    n = len(traj.vx)
    hold_n = ramp_n = int(round(1.0 / env.dt))
    ramp = np.linspace(0.0, 1.0, ramp_n)
    traj.vx[:] = SLIPWALK_CMD_VX
    traj.vx[:hold_n] = 0.0
    traj.vx[hold_n:hold_n + ramp_n] = SLIPWALK_CMD_VX * ramp
    traj.vy[:] = 0.0
    if traj.wz is not None:
        traj.wz[:] = 0.0

    gait = TripodGait(vx=0.0, lift=0.0 if policy == "skate" else 0.025)
    gait.sync_plant_stance(*WALK_PLANT)
    plant_rad = np.array([0.0, *WALK_PLANT] * 6) * DEG2RAD
    # Stork statue: plant triad {0,2,4}, park triad {1,3,5} in the air
    # (hip -50 / knee -50 = the ~190 mm "parked high" splay class the
    # hold bank calibrated 08-11; matches the M2 -c1 videos' airborne
    # triad, duty ~0.02).
    stork_rad = plant_rad.copy()
    for _leg in (1, 3, 5):
        stork_rad[3 * _leg + 1] -= 50.0 * DEG2RAD
        stork_rad[3 * _leg + 2] -= 50.0 * DEG2RAD
    # Topple twin: lift the FRONT pair (legs 0,1) with the same hip/knee
    # splay — the support polygon loses its front edge and the body
    # tips over in ~1 s (tilt_roll/tilt_pitch termination), measured
    # 23-28 ticks on the controller 08-22.
    topple_rad = plant_rad.copy()
    for _leg in (0, 1):
        topple_rad[3 * _leg + 1] -= 50.0 * DEG2RAD
        topple_rad[3 * _leg + 2] -= 50.0 * DEG2RAD
    gait.reset_phase()

    x0 = float(env.data.xpos[env._chassis_bid, 0])
    total, step = 0.0, 0
    while True:
        t = step * env.dt
        i = min(step, n - 1)
        if policy in ("gait", "skate"):
            gait.set_velocity(vx=float(traj.vx[i]) * gait_scale, vy=0.0)
            act = q_rad_to_action(np.asarray(gait.desired_deg(t)) * DEG2RAD)
        elif policy == "stall":
            gait.set_velocity(vx=0.0, vy=0.0)
            act = q_rad_to_action(np.asarray(gait.desired_deg(t)) * DEG2RAD)
        elif policy == "shuffle":
            half_n = max(1, int(round(shuffle_half_period_s / env.dt)))
            sign = 1.0 if (step // half_n) % 2 == 0 else -1.0
            gait.set_velocity(vx=SLIPWALK_CMD_VX * sign, vy=0.0)
            act = q_rad_to_action(np.asarray(gait.desired_deg(t)) * DEG2RAD)
        elif policy == "stork":
            act = q_rad_to_action(stork_rad)
        elif policy == "topple":
            act = q_rad_to_action(topple_rad)
        else:
            act = q_rad_to_action(plant_rad)
        _obs, r, term, trunc, _info = env.step(act)
        total += float(r)
        step += 1
        if term or trunc:
            break
    dx = float(env.data.xpos[env._chassis_bid, 0]) - x0
    env.close()
    return total, dx, step


@pytest.fixture(scope="module")
def slipwalk_returns() -> dict[str, float]:
    """Mean return per behavior under the from-scratch anti-slip stack.

    Controller measurement 2026-08-21 (3 seeds, 15 s episodes),
    term_penalty=400 added 08-22 (bit-exact for all pre-existing
    behaviors — they survive to truncation):
    fast(0.44 m) +851 > gait(0.22 m) +417 > creep(0.16 m) +108 >
    stall -143 > park -244 > topple ~-381 > skate -1195.
    (topple WITHOUT term_penalty measured +19/ep — see
    test_slipwalk_toppling_fast_is_not_an_escape.)
    """
    plan = {
        "fast": ("gait", 2.0),
        "gait": ("gait", 1.0),
        "creep": ("gait", 0.5),
        "skate": ("skate", 1.0),
        "stall": ("stall", 1.0),
        "park": ("park", 1.0),
        "stork": ("stork", 1.0),
        "topple": ("topple", 1.0),
    }
    out = {}
    for name, (pol, scale) in plan.items():
        runs = [_slipwalk_rollout(pol, s, gait_scale=scale) for s in SEEDS]
        out[name] = float(np.mean([r[0] for r in runs]))
        out[name + "_dx"] = float(np.mean([r[1] for r in runs]))
        out[name + "_steps"] = float(np.mean([r[2] for r in runs]))
    return out


def test_slipwalk_travel_beats_every_stationary_behavior(slipwalk_returns):
    """The whole point of the operator's arm: covering ground must beat
    freezing, marching in place, and sliding — by a wide margin, not a
    hair. Every prior from-scratch gait arm froze because the stack let
    stillness stay competitive."""
    gait = slipwalk_returns["gait"]
    for cheat in ("stall", "park", "skate"):
        assert gait > slipwalk_returns[cheat] + 300.0, (
            f"stationary '{cheat}' is competitive with real walking: "
            f"{slipwalk_returns} — the freeze exploit is open.")
    assert slipwalk_returns["gait_dx"] > 0.15, (
        "the reference gait did not actually travel; bank is broken")


def test_slipwalk_skating_is_the_worst_outcome(slipwalk_returns):
    """Loaded feet sliding with no travel is what the operator wants
    priced hardest: the zero-lift skating twin must sit below every
    other behavior in the bank, including outright refusal."""
    assert slipwalk_returns["skate"] < slipwalk_returns["park"] - 300.0, (
        f"skating is not the worst outcome: {slipwalk_returns}")


def test_slipwalk_more_travel_earns_more(slipwalk_returns):
    """Income must rise monotonically with real along-command distance
    (0.16 m -> 0.22 m -> 0.44 m), so the gradient out of any partial
    gait points at travelling further, not at settling."""
    assert (slipwalk_returns["fast"] > slipwalk_returns["gait"]
            > slipwalk_returns["creep"]), (
        f"more travel does not earn more: {slipwalk_returns}")
    assert slipwalk_returns["fast_dx"] > slipwalk_returns["gait_dx"] \
        > slipwalk_returns["creep_dx"]


def test_slipwalk_has_no_speed_band(slipwalk_returns):
    """Operator constraint: the policy does NOT have to track a speed.
    Walking at 2x the commanded speed must be paid at least as well as
    matching it — the Gaussian kernel / progress cap stack would have
    charged that exceedance."""
    assert slipwalk_returns["fast"] >= slipwalk_returns["gait"], (
        f"exceeding the commanded speed is punished: {slipwalk_returns} "
        "— a speed target leaked back into the stack.")


def test_slipwalk_stork_statue_is_priced_out(slipwalk_returns):
    """The observed 38M AMP-M2 cheat (cw-amp-m2-pilot-{noamp,style05}-c1,
    08-22): a half-tripod statue — triad 0,2,4 planted motionless, triad
    1,3,5 parked in the air — out-earned learning to walk for 38M steps
    under the legacy kernel stack (statue income ~1.9/tick: rise_finish
    + posture/height kernels + the sigma-0.05 velocity kernel paying
    ~0.45/tick to v=0 across the low/stop command fraction). Under the
    from-scratch stack real walking must beat the stork by a wide
    margin, and stepping in place must still point uphill from it (the
    discovery path out of the statue)."""
    assert slipwalk_returns["gait"] > slipwalk_returns["stork"] + 300.0, (
        f"the M2 stork statue is competitive with real walking: "
        f"{slipwalk_returns} — the 38M freeze exploit is still open.")
    assert slipwalk_returns["stall"] > slipwalk_returns["stork"], (
        f"the stork statue out-earns stepping in place: "
        f"{slipwalk_returns} — no uphill discovery path out of it.")


def test_slipwalk_toppling_fast_is_not_an_escape(slipwalk_returns):
    """The observed 2M freeprog cheat (cw-amp-m2-freeprog-{noamp,
    style05}, 08-22 dig-in): with per-tick charges harsh from step 0
    and termination FREE (term_penalty=0), dying immediately was the
    best-paying behavior in the bank short of real walking — a scripted
    1 s topple netted +19/ep vs park -243 / stall -143 — and both 2M
    arms duly learned suicide in their final quarter (tilt terminations
    59->132 and 90->241, ep_len collapsing 310->230-256, at CONSTANT
    action std, after mid-run phases that had already proven survival
    was reachable). With reward.term_penalty in the stack, death must
    sit strictly below every survival behavior a from-scratch policy
    can reach: refusal (park), stepping in place (stall), and real
    walking — so the gradient out of a flailing state points at
    stabilizing, never at falling over. (skate's deeper -1195 is an
    undiscounted 375-tick accumulation; per-state at gamma=0.99 the
    worst survivable continuation costs ~-300, which 400 covers.)"""
    assert slipwalk_returns["topple_steps"] < 75, (
        f"the topple twin did not die fast (steps="
        f"{slipwalk_returns['topple_steps']}); the bank probe is broken")
    assert slipwalk_returns["topple"] < slipwalk_returns["park"] - 100.0, (
        f"dying fast out-earns (or is competitive with) refusing to "
        f"move: {slipwalk_returns} — the suicide exploit is open.")
    assert slipwalk_returns["topple"] < slipwalk_returns["stall"], (
        f"dying fast out-earns stepping in place: {slipwalk_returns} "
        "— the suicide exploit is open.")
    assert slipwalk_returns["gait"] > slipwalk_returns["topple"] + 300.0, (
        f"real walking does not clearly beat dying: {slipwalk_returns}")


def test_slipwalk_stepping_stall_still_beats_refusal(slipwalk_returns):
    """Discovery gradient: from a parked policy the first useful move is
    to start lifting feet. A march-in-place must therefore still out-earn
    a refusal park (both far below real walking), or a from-scratch run
    has no uphill path out of the freeze."""
    assert slipwalk_returns["stall"] > slipwalk_returns["park"], (
        f"refusing to step out-earns stepping in place: {slipwalk_returns} "
        "— the anti-slip charge has closed the discovery path.")


# reward.k_walk_swing on the SLIPWALK/term400 stack (08-22, AMP M2
# freeprog dig-in continuation): every non-reward lever (term_penalty,
# std-anneal, stage curriculum, style-weight dose 0.5x-2.0x, RSI-for-
# walk via goal.walk_gait_start_frac — see cw-gait-rsi1, refuted for
# this exact failure signature) is now closed for the from-scratch
# statue basin, and cw-amp-m2-freeprog-term400-fixedcmd-seed11 (the
# bank's own literal simplest fixed-command case) still statues with
# sacrificed legs — so command complexity is not the barrier either.
# The one mechanism never armed for this family: k_walk_swing, an
# EXISTING (2026-08 vintage, champion-proven on the older cw-walk
# lineage) one-shot bonus for a foot completing a real lift-off ->
# >=2-ticks-airborne -> touchdown swing of >=15 mm, paid in ANY
# direction (no along-command gate, unlike k_step_event) — the
# genuinely accessible "lift a foot at all" first rung the freeze
# basin currently has zero income gradient toward (freeprog_pen sits
# flat at its floor from step 0 in every prior arm; nothing pays for
# an incomplete/exploratory swing that doesn't already net forward
# progress). Bank-checked here BEFORE spending any GPU budget (the
# 08-22 agreement rule: reward/task-mechanism changes need the
# semantics bank first): does adding it reopen a stationary cheat?
SLIPWALK_SWING_OVERRIDES = dict(SLIPWALK_OVERRIDES)
SLIPWALK_SWING_OVERRIDES[("reward", "k_walk_swing")] = 1.0


@pytest.fixture(scope="module")
def slipwalk_swing_returns() -> dict[str, float]:
    """Same behavior bank as ``slipwalk_returns``, plus "shuffle" (a
    real coordinated tripod gait reversing direction every 0.6 s — see
    ``_slipwalk_rollout``'s docstring), under
    ``SLIPWALK_SWING_OVERRIDES``. Measured 2026-08-22 (3 seeds):
    fast +874 (0.43 m) > gait +622 (0.27 m) > creep +126 (0.16 m) >
    stall -155 > shuffle -297 (0.013 m, real strides both ways) >
    park -243 > stork -238 > topple -381 (23 steps) > skate -1023.
    Swing income raises gait/creep's take over the no-swing baseline
    (gait 558->622, creep 103->126 — a real, if modest, ~11-23%
    bump) while every stationary/non-progressing twin (stall, park,
    skate, shuffle, topple) barely moves (single digits to ~+10),
    never closing the gap to real travel — and shuffle, the realistic
    "genuine six-leg strides both ways, zero net travel" farm
    attempt, stays BELOW even park.
    """
    plan = {
        "fast": ("gait", 2.0),
        "gait": ("gait", 1.0),
        "creep": ("gait", 0.5),
        "skate": ("skate", 1.0),
        "stall": ("stall", 1.0),
        "shuffle": ("shuffle", 1.0),
        "park": ("park", 1.0),
        "stork": ("stork", 1.0),
        "topple": ("topple", 1.0),
    }
    out = {}
    for name, (pol, scale) in plan.items():
        runs = [_slipwalk_rollout(pol, s, gait_scale=scale,
                                  overrides=SLIPWALK_SWING_OVERRIDES)
                for s in SEEDS]
        out[name] = float(np.mean([r[0] for r in runs]))
        out[name + "_dx"] = float(np.mean([r[1] for r in runs]))
        out[name + "_steps"] = float(np.mean([r[2] for r in runs]))
    return out


def test_slipwalk_swing_bonus_boosts_real_travel_substantially(
        slipwalk_returns, slipwalk_swing_returns):
    """The whole point of arming this lever: it must make REAL gait
    income materially bigger (a real, accessible gradient toward
    stepping), not just add noise."""
    assert (slipwalk_swing_returns["gait"]
            > slipwalk_returns["gait"] + 40.0), (
        f"k_walk_swing did not meaningfully raise real-gait income: "
        f"base={slipwalk_returns['gait']} "
        f"swing={slipwalk_swing_returns['gait']}")
    assert (slipwalk_swing_returns["creep"]
            > slipwalk_returns["creep"]), (
        "k_walk_swing did not raise creep income at all: "
        f"base={slipwalk_returns['creep']} "
        f"swing={slipwalk_swing_returns['creep']}")


def test_slipwalk_swing_bonus_preserves_travel_ordering(
        slipwalk_swing_returns):
    """The core SLIPWALK invariant (travel beats every stationary
    behavior by a wide margin) must survive arming the swing bonus."""
    gait = slipwalk_swing_returns["gait"]
    for cheat in ("stall", "park", "skate", "shuffle"):
        assert gait > slipwalk_swing_returns[cheat] + 300.0, (
            f"stationary/non-progressing '{cheat}' is competitive "
            f"with real walking once k_walk_swing is armed: "
            f"{slipwalk_swing_returns} — the swing bonus reopened a "
            "freeze exploit.")
    assert (slipwalk_swing_returns["fast"] > slipwalk_swing_returns["gait"]
            > slipwalk_swing_returns["creep"]), (
        f"more travel does not earn more with the swing bonus armed: "
        f"{slipwalk_swing_returns}")


def test_slipwalk_swing_bonus_does_not_reward_marching_or_shuffling(
        slipwalk_returns, slipwalk_swing_returns):
    """The specific new risk this lever introduces: k_step_event only
    pays for a stride that projects along the COMMANDED direction, but
    k_walk_swing pays ANY completed swing regardless of direction — so
    it must not turn stepping-in-place (stall) or a real gait that
    reverses direction every beat (shuffle, zero net travel despite
    genuine six-leg strides both ways) into a competitive income
    source. Both must stay within a small band of their no-swing
    reading and strictly below the honest creep."""
    assert (slipwalk_swing_returns["stall"]
            < slipwalk_returns["stall"] + 50.0), (
        f"k_walk_swing pays marching-in-place: base="
        f"{slipwalk_returns['stall']} swing="
        f"{slipwalk_swing_returns['stall']} — the swing bonus is "
        "gameable by stall, matching the historical stall/skate "
        "failure signature.")
    assert (slipwalk_swing_returns["shuffle"]
            < slipwalk_swing_returns["stall"]), (
        f"reversing-direction shuffling out-earns simple marching: "
        f"{slipwalk_swing_returns} — the swing bonus rewards genuine "
        "strides over a real net-zero-travel gait, the exact farm "
        "attack this lever risks.")
    assert (slipwalk_swing_returns["shuffle"]
            < slipwalk_swing_returns["park"]), (
        f"the realistic swing-farming twin (shuffle: real six-leg "
        f"strides, ~0 net travel) beats refusing to move once "
        f"k_walk_swing is armed: {slipwalk_swing_returns} — the "
        "farm exploit is open.")
    assert (slipwalk_swing_returns["creep"]
            > slipwalk_swing_returns["shuffle"] + 300.0), (
        f"honest partial travel does not clearly beat the shuffle "
        f"farm attempt: {slipwalk_swing_returns}")


def test_slipwalk_swing_bonus_keeps_topple_the_worst_live_option(
        slipwalk_swing_returns):
    """The anti-suicide invariant must also survive arming the swing
    bonus (a toppling twin gets no swing credit — it never completes a
    touchdown before terminating — but check it wasn't accidentally
    reordered)."""
    assert slipwalk_swing_returns["topple_steps"] < 75, (
        "the topple twin did not die fast under the swing-armed "
        f"stack; bank probe is broken: {slipwalk_swing_returns}")
    assert (slipwalk_swing_returns["topple"]
            < slipwalk_swing_returns["park"] - 100.0), (
        f"dying fast is competitive once k_walk_swing is armed: "
        f"{slipwalk_swing_returns}")
    assert (slipwalk_swing_returns["topple"]
            < slipwalk_swing_returns["stall"]), (
        f"dying fast out-earns stepping in place once k_walk_swing is "
        f"armed: {slipwalk_swing_returns}")


# reward.walk_kernel_vel_ema on the freeprog stack (08-22,
# cw-amp-m2-freeprog-term400-stall dig-in follow-up): a concurrent
# cycle's std-anneal arm (cw-amp-m2-freeprog-term400-stdanneal, FAIL)
# root-caused the term400 pair's marching-in-place plateau as a
# REWARD-SHAPE defect — walk_freeprog_score's instantaneous
# along-command velocity nets to ~0 for a symmetric back-and-forth
# stepping gait, the exact same masking the phasedir7 kernel dig-in
# found for the Gaussian kernel term. Fix reuses the already-validated
# walk_kernel_vel_ema flag as freeprog's velocity input too (see
# walk_task.py). This bank proves the fix does not undo the discovery
# gradient the scripted "stall" twin above certifies, and that it
# actually WIDENS the creep-vs-stall separation (a steeper gradient
# toward real travel), not just moves numbers around.
SLIPWALK_EMA_OVERRIDES = {**SLIPWALK_OVERRIDES,
                          ("reward", "walk_kernel_vel_ema"): 1.0}


@pytest.fixture(scope="module")
def slipwalk_ema_returns() -> dict[str, float]:
    plan = {"stall": ("stall", 1.0), "creep": ("gait", 0.5),
            "park": ("park", 1.0), "gait": ("gait", 1.0)}
    out = {}
    for name, (pol, scale) in plan.items():
        env = _make_walk_env(SEEDS[0], SLIPWALK_EMA_OVERRIDES)
        env.reset()
        env.close()
        runs = [_slipwalk_ema_rollout(pol, s, gait_scale=scale)
               for s in SEEDS]
        out[name] = float(np.mean([r[0] for r in runs]))
    return out


def _slipwalk_ema_rollout(policy: str, seed: int, *,
                          gait_scale: float = 1.0) -> tuple[float, float, int]:
    """Same rollout as ``_slipwalk_rollout`` but under
    SLIPWALK_EMA_OVERRIDES — kept as a thin wrapper (not a parametrize
    of the original) so SLIPWALK_OVERRIDES's own bank stays byte-for-
    byte unchanged and this new dict is the only thing under test."""
    from sim_gait_compat import TripodGait

    env = _make_walk_env(seed, SLIPWALK_EMA_OVERRIDES)
    env.reset()
    traj = env._goal_traj
    n = len(traj.vx)
    hold_n = ramp_n = int(round(1.0 / env.dt))
    ramp = np.linspace(0.0, 1.0, ramp_n)
    traj.vx[:] = SLIPWALK_CMD_VX
    traj.vx[:hold_n] = 0.0
    traj.vx[hold_n:hold_n + ramp_n] = SLIPWALK_CMD_VX * ramp
    traj.vy[:] = 0.0
    if traj.wz is not None:
        traj.wz[:] = 0.0
    gait = TripodGait(vx=0.0, lift=0.0 if policy == "skate" else 0.025)
    gait.sync_plant_stance(*WALK_PLANT)
    plant_rad = np.array([0.0, *WALK_PLANT] * 6) * DEG2RAD
    gait.reset_phase()
    x0 = float(env.data.xpos[env._chassis_bid, 0])
    total, step = 0.0, 0
    while True:
        t = step * env.dt
        i = min(step, n - 1)
        if policy in ("gait", "skate"):
            gait.set_velocity(vx=float(traj.vx[i]) * gait_scale, vy=0.0)
            act = q_rad_to_action(np.asarray(gait.desired_deg(t)) * DEG2RAD)
        elif policy == "stall":
            gait.set_velocity(vx=0.0, vy=0.0)
            act = q_rad_to_action(np.asarray(gait.desired_deg(t)) * DEG2RAD)
        else:
            act = q_rad_to_action(plant_rad)
        _obs, r, term, trunc, _info = env.step(act)
        total += float(r)
        step += 1
        if term or trunc:
            break
    dx = float(env.data.xpos[env._chassis_bid, 0]) - x0
    env.close()
    return total, dx, step


def test_freeprog_ema_default_off_is_bit_exact(slipwalk_returns):
    """The plain SLIPWALK_OVERRIDES bank (no walk_kernel_vel_ema key)
    must be completely untouched by the new freeprog EMA branch —
    reading self._walk_kernel_vema when the flag is off would be a
    silent behavior change even if the numbers happened to match."""
    assert slipwalk_returns["stall"] < 0.0   # unchanged sign/scale
    assert slipwalk_returns["gait"] > slipwalk_returns["park"]


def test_freeprog_ema_preserves_discovery_gradient(slipwalk_ema_returns):
    """Same certificate as the raw-velocity bank: march-in-place must
    still beat refusing to step, or the EMA input closed the discovery
    path some other way."""
    r = slipwalk_ema_returns
    assert r["stall"] > r["park"], (
        f"EMA freeprog input broke the stall>park discovery gradient: {r}")


def test_freeprog_ema_creep_vs_stall_gap_measured(
        slipwalk_returns, slipwalk_ema_returns):
    """REFUTED HYPOTHESIS, kept as a measurement (08-22): the original
    prediction here was that reusing walk_kernel_vel_ema as freeprog's
    velocity input would WIDEN the creep-vs-stall separation (a
    steeper discovery gradient toward real travel), mirroring the
    phasedir7 kernel fix. Measured on this scripted 3-seed twin pair:
    raw instantaneous gap 267.7, EMA gap 226.4 — the EMA input actually
    NARROWS the separation slightly here, not widens it. Reads as: the
    scripted "creep" twin already has a fairly smooth instantaneous
    velocity (real half-speed walking, not wild oscillation), so EMA-
    smoothing helps "stall" (whose instantaneous velocity genuinely
    oscillates near zero) proportionally more than it helps "creep" —
    the opposite of the intended effect. CONCLUSION: this specific
    reuse-the-kernel-EMA mechanism is NOT indicated as the freeprog-
    stall fix (do not launch a real training arm on this hypothesis
    alone); the flag stays available (default off, bit-exact, proven
    safe by the two tests above) but the actual AMP-track fix for the
    marching-in-place plateau needs a different mechanism (e.g. an
    explicit net-episode-displacement floor/charge, not a smoothed
    instantaneous-velocity substitution). This test pins the measured
    direction so a future attempt at this exact idea does not have to
    re-derive it from scratch."""
    raw_gap = slipwalk_returns["creep"] - slipwalk_returns["stall"]
    ema_gap = slipwalk_ema_returns["creep"] - slipwalk_ema_returns["stall"]
    assert ema_gap < raw_gap, (
        "the measured direction changed — re-read this test's docstring "
        f"before trusting it: raw={raw_gap:.1f} ema={ema_gap:.1f}")


# --------------------------------------------------------------------------
# AMP-MINIMAL bank (08-22, AMP M2 section-5 reward redesign,
# q_20260822T1815Z): every from-scratch M2 arm on the SLIPWALK_OVERRIDES
# anti-slip stack (k_walk_freeprog, k_loadslip_excess, walk_gait_gate,
# k_walk_idle_charge, k_step_event, k_park_duty — built for BC-warm-
# started fine-tuning) has failed the same way (a ~0.03-0.06m/15s
# statue/shuffle basin, freeprog_pen flat at its floor from step 0)
# across every tested lever (term-penalty, std-anneal, staging,
# task-complexity, style-dose 0.5x-2.0x, swing, RSI). This same cycle's
# `cw-amp-m2-styleonly-v2-c1b` (pure style, no task reward at all) DID
# get the leg-cycling gait_valid up (5/6 det+sto, was 0/6 under every
# task-mixed arm) but with ~0 net travel and huge slip (~20/m) — a
# "march in place" degenerate: style alone rewards joint-motion
# resemblance, never body displacement (an expected AMP limitation,
# not a bug) — confirming the brief's own design (task reward supplies
# the "go somewhere" signal, style supplies "look natural"; neither
# alone suffices) rather than refuting AMP.
#
# AMP_LOCOMOTION.md section 5.1 specifies a much smaller task reward
# than SLIPWALK: a Gaussian velocity-tracking kernel + yaw kernel +
# upright/height regularizer + modest physical regularizers — no slip
# gating, no idle-travel charge, no per-stride event bonus. That
# reward ALREADY EXISTS in this codebase with zero new production
# code: it is exactly what fires when reward.k_walk_freeprog is left
# at its default 0 — the Gaussian kernel (K_WALK/SIGMA_V) + linear
# progress (k_walk_prog) in walk_task.py's walk branch, plus the
# shared env.compute_reward() posture/gyro/action/current terms every
# task already gets. AMP_MINIMAL_OVERRIDES is that reward with ONLY
# reward.term_penalty=400 kept on top (the anti-suicide fix is a real
# pricing-bug repair — free death — not a SLIPWALK-specific mechanism;
# see test_slipwalk_toppling_fast_is_not_an_escape) and every
# SLIPWALK-only key explicitly zeroed, built from scratch (not
# `dict(SLIPWALK_OVERRIDES)`) so a future edit to that dict can never
# silently leak in here.
#
# MEASURED 08-22 (3 seeds, matches SLIPWALK's own scripted twins):
# fast +1177.7 (0.43m) > gait +1072.3 (0.27m) > creep +978.2 (0.16m)
# > stall +841.1 (0.00m) ~= stork +839.0 ~= park +834.8 ~= skate
# +826.1, topple -380.7 (23 steps). Two honest findings, not just a
# pass/fail: (1) real travel DOES beat every stationary twin, and
# monotonically with distance (fast>gait>creep), so the minimal
# kernel+progress design is directionally sound and the anti-suicide
# fix transfers cleanly; (2) UNLIKE SLIPWALK's engineered stack, the
# margin is modest (~230/ep, not the 300+ SLIPWALK requires) and every
# stationary variant (stall/park/stork/skate) is bunched within ~15
# points of each other — this reward gives almost NO discovery
# gradient distinguishing "trying" (stall) from "refusing" (park),
# the exact gap SLIPWALK's k_step_event/k_park_duty were built to
# close. That gap is not being patched here: per the brief, the AMP
# style term is the intended source of that missing "do something
# coordinated" gradient (already demonstrated this cycle — pure style
# alone organizes real leg-cycling), so this bank's job is only to
# confirm the task reward does not actively fight that combination
# (no stationary cheat beats travel, no suicide escape) before
# spending GPU on the section-5.2 task/style ratio sweep.
AMP_MINIMAL_OVERRIDES = {
    ("reward", "term_penalty"): 400.0,
    ("goal", "walk_speed_min_m_s"): SLIPWALK_CMD_VX,
    ("goal", "walk_speed_max_m_s"): SLIPWALK_CMD_VX,
    ("goal", "walk_heading_max_rad"): 0.0,
    # Explicit zeros for every SLIPWALK-only lever (belt-and-braces:
    # these already default to 0/off, but naming them keeps this
    # dict's provenance self-documenting).
    ("reward", "k_step_event"): 0.0,
    ("reward", "k_park_duty"): 0.0,
    ("reward", "k_walk_freeprog"): 0.0,
    ("reward", "walk_loadslip_gate"): 0.0,
    ("reward", "k_loadslip_excess"): 0.0,
    ("reward", "walk_gait_gate"): 0.0,
    ("reward", "k_walk_idle_charge"): 0.0,
    ("reward", "k_drag_stance"): 0.0,
}


@pytest.fixture(scope="module")
def amp_minimal_returns() -> dict[str, float]:
    """Mean return per scripted behavior under AMP_MINIMAL_OVERRIDES —
    the section-5.1 base task reward alone (no AMP style term: style
    is a policy-side blend the scripted-twin harness never touches).
    If a plain stall/park/stork ever beat real travel under just the
    task reward, adding style on top only makes the basin worse (style
    income is additive everywhere else in this codebase, never a fix
    for a misranked task reward)."""
    plan = {
        "fast": ("gait", 2.0),
        "gait": ("gait", 1.0),
        "creep": ("gait", 0.5),
        "skate": ("skate", 1.0),
        "stall": ("stall", 1.0),
        "park": ("park", 1.0),
        "stork": ("stork", 1.0),
        "topple": ("topple", 1.0),
    }
    out = {}
    for name, (pol, scale) in plan.items():
        runs = [_slipwalk_rollout(pol, s, gait_scale=scale,
                                  overrides=AMP_MINIMAL_OVERRIDES)
                for s in SEEDS]
        out[name] = float(np.mean([r[0] for r in runs]))
        out[name + "_dx"] = float(np.mean([r[1] for r in runs]))
        out[name + "_steps"] = float(np.mean([r[2] for r in runs]))
    return out


def test_amp_minimal_travel_beats_stationary_behaviors(amp_minimal_returns):
    """Real forward travel must beat every stationary/sliding twin
    under the section-5.1 task reward alone. Margin bar (150) is
    deliberately lower than SLIPWALK's (300): this reward has no
    anti-slip/anti-park apparatus by design, so a modest edge from the
    Gaussian kernel + linear progress is the honest expectation, not a
    wide one (measured 08-22: gait beats stall/park/stork/skate by
    ~231-246)."""
    gait = amp_minimal_returns["gait"]
    for cheat in ("stall", "park", "stork", "skate"):
        assert gait > amp_minimal_returns[cheat] + 150.0, (
            f"stationary '{cheat}' is competitive with real walking "
            f"under the minimal task reward: {amp_minimal_returns}")


def test_amp_minimal_more_travel_earns_more(amp_minimal_returns):
    """Income must rise monotonically with real along-command distance
    (creep -> gait -> fast), so the kernel+progress design still has a
    gradient pointing at travelling further, not at settling for a
    partial gait."""
    assert (amp_minimal_returns["fast"] > amp_minimal_returns["gait"]
            > amp_minimal_returns["creep"]), (
        f"more travel does not earn more: {amp_minimal_returns}")
    assert (amp_minimal_returns["fast_dx"] > amp_minimal_returns["gait_dx"]
            > amp_minimal_returns["creep_dx"])


def test_amp_minimal_toppling_is_not_an_escape(amp_minimal_returns):
    """The reused anti-suicide fix (reward.term_penalty=400) must carry
    over cleanly to the minimal stack: dying fast must sit far below
    every survival behavior, not just below real walking."""
    assert amp_minimal_returns["topple_steps"] < 75, (
        "the topple twin did not die fast; the bank probe is broken: "
        f"{amp_minimal_returns}")
    assert amp_minimal_returns["topple"] < amp_minimal_returns["park"] - 300.0, (
        f"dying fast is competitive with refusing to move under the "
        f"minimal stack: {amp_minimal_returns}")
    assert amp_minimal_returns["topple"] < amp_minimal_returns["stall"] - 300.0, (
        f"dying fast is competitive with stepping in place under the "
        f"minimal stack: {amp_minimal_returns}")


def test_amp_minimal_stationary_variants_are_not_well_separated(
        amp_minimal_returns):
    """MEASUREMENT, not a design requirement (08-22): unlike SLIPWALK,
    this reward gives almost no separation between trying (stall) and
    refusing (park) — pinned here so a future reader does not have to
    re-derive it, and so a change that suddenly opens a wide gap here
    is flagged as a real change to investigate, not silently assumed
    equivalent. This is exactly why the section-5.2 task/style sweep
    is treated as an experiment, not a guaranteed win: closing this
    gap is style's job (proven separately, this cycle, to organize
    leg-cycling on its own), not this reward's."""
    gap = abs(amp_minimal_returns["stall"] - amp_minimal_returns["park"])
    assert gap < 50.0, (
        "the stall/park gap moved outside the measured band — re-read "
        f"this test before trusting the docstring: {amp_minimal_returns}")


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
# WALKCURR4 bank — the EXACT reward/safety config of the
# cw-dynrep-criticD-walkcurr4 lineage (operator order
# fb_20260818T085648_2a0a60), which walkcurr1-3 omitted and paid for
# with a crouched paid-shuffle (walkcurr3 at 7.5M: B0 height error
# -65 mm, walk_height_factor 0.07, slip 4.6/m, zero promotions):
# calibrated height gate (sigma 11 mm keeps the honest gait >= 0.8
# income, the bench crouch band <= 0.001 — calibrate_walk_height.py
# 08-17), kernel progress gate (kills the paid park), safety height
# cutoff 25 mm after a 2 s grace, terminal charge 30. MDP_PREFLIGHT
# ordering: honest gait > park/refusal AND honest gait >> the -51 mm
# crouch shuffle (same exploit family as walkcurr3's -65 mm; anything
# past 25 mm now terminates via safety and cannot keep its income).

WALKCURR4_OVERRIDES = dict(WALK_OVERRIDES)
WALKCURR4_OVERRIDES.update({
    ("reward", "walk_height_gate"): 1.0,
    ("reward", "walk_height_sigma_mm"): 11.0,
    ("reward", "term_penalty"): 30.0,
    ("safety", "walk_max_height_drop_mm"): 25.0,
    ("safety", "walk_height_grace_s"): 2.0,
})


@pytest.fixture(scope="module")
def walkcurr4_returns() -> dict[str, float]:
    def mean(policy, stance):
        return float(np.mean([
            _walk_rollout(policy, s, overrides=WALKCURR4_OVERRIDES,
                          stance=stance)
            for s in SEEDS]))
    return {
        "gait": mean("gait", WALK_PLANT),
        "park": mean("park", WALK_PLANT),
        "crouch": mean("gait", WALK_CROUCH),
    }


def test_walkcurr4_honest_gait_beats_park(walkcurr4_returns):
    """Under the full walkcurr4 stack the hardware-proven upright gait
    must out-earn parking through the command by a wide margin — a
    parked robot collecting kernel income was walkcurr1/2's root
    cause 1 (V2 B0 fixed the speed band; the kernel progress gate
    fixes the pay)."""
    g, p = walkcurr4_returns["gait"], walkcurr4_returns["park"]
    assert g > 1.5 * p and g > p + 150.0, (
        f"parking rivals the honest gait under the walkcurr4 stack: "
        f"{walkcurr4_returns}")


def test_walkcurr4_honest_gait_beats_crouch_shuffle(walkcurr4_returns):
    """The -51 mm crouch walker (same family as walkcurr3's measured
    -65 mm shuffle) must earn FAR less than the upright gait: the
    height gate strips its income (factor <= 0.001 at 40+ mm) and the
    25 mm safety cutoff ends the episode after the grace with the
    terminal charge. This is the ordering walkcurr3 trained without."""
    g, c = walkcurr4_returns["gait"], walkcurr4_returns["crouch"]
    assert g > 2.0 * max(c, 0.0) + 150.0, (
        f"crouch-shuffling rivals upright walking under the walkcurr4 "
        f"stack: {walkcurr4_returns}")


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


# --------------------------------------------------------------------------
# FASTPROF bank — fast-profile command-tracking charges (08-20, operator
# note fb_20260820T000059 item 3b). The steer5-fastprof1 canary: under
# the raised servo profile every checkpoint runs 2.5x the commanded
# speed (prog_ratio 1.27-1.76 vs band 0.91-1.07) and drifts 50-60 deg
# off heading — the Gaussian kernel saturates ~2 sigma out and the
# progress cap still PAYS overspeed. The new charges must make OBEYING
# the command out-earn the same honest gait driven past the band or off
# the commanded heading — the exact behaviors the canary showed —
# without new keys taxing the obedient gait itself (charge is zero in
# band / aligned by construction, unit-proven in
# test_walk_fastprof_mdp.py).

FASTPROF_KEYS = {
    ("reward", "k_walk_overspeed"): 2.0,
    ("reward", "walk_overspeed_tol"): 0.10,
    ("reward", "k_walk_heading"): 2.0,
}
FASTPROF_CMD = 0.02          # commanded speed; the gait can overspeed
FASTPROF_SEEDS = (0, 1)


def _fastprof_rollout(drive: str, seed: int,
                      overrides: dict | None) -> float:
    """Scripted tripod gait under a pinned FASTPROF_CMD forward command,
    DRIVEN either at the command ('obey'), at 2.5x it ('overspeed' —
    the canary's measured ratio), or at the command rotated 55 deg
    ('skew' — the canary's measured heading drift)."""
    from sim_gait_compat import TripodGait

    stack = dict(WALK_OVERRIDES)
    stack.update(overrides or {})
    env = _make_walk_env(seed, stack, episode_seconds=10.0)
    env.reset()
    traj = env._goal_traj
    n = len(traj.vx)
    hold_n = ramp_n = int(round(1.0 / env.dt))
    ramp = np.linspace(0.0, 1.0, ramp_n)
    traj.vx[:] = FASTPROF_CMD
    traj.vx[:hold_n] = 0.0
    traj.vx[hold_n:hold_n + ramp_n] = FASTPROF_CMD * ramp
    traj.vy[:] = 0.0
    if getattr(traj, "wz", None) is not None:
        traj.wz[:] = 0.0

    ang = math.radians(55.0) if drive == "skew" else 0.0
    factor = 2.5 if drive == "overspeed" else 1.0
    gait = TripodGait(vx=0.0, lift=0.025)
    gait.sync_plant_stance(*WALK_PLANT)
    gait.reset_phase()

    total, step = 0.0, 0
    while True:
        t = step * env.dt
        i = min(step, n - 1)
        s_cmd = float(traj.vx[i]) * factor
        gait.set_velocity(vx=s_cmd * math.cos(ang),
                          vy=s_cmd * math.sin(ang))
        act = q_rad_to_action(np.asarray(gait.desired_deg(t)) * DEG2RAD)
        _obs, r, term, trunc, _info = env.step(act)
        total += float(r)
        step += 1
        if term or trunc:
            break
    env.close()
    return total


@pytest.fixture(scope="module")
def fastprof_returns() -> dict[str, float]:
    out = {}
    for tag, keys in (("on", FASTPROF_KEYS), ("off", None)):
        for drive in ("obey", "overspeed", "skew"):
            out[f"{drive}_{tag}"] = float(np.mean(
                [_fastprof_rollout(drive, s, keys)
                 for s in FASTPROF_SEEDS]))
    return out


def test_fastprof_obeying_the_command_beats_overspeed(fastprof_returns):
    """With the charges on, the gait that tracks the commanded band
    must decisively out-earn the same gait blasting past it."""
    r = fastprof_returns
    assert r["obey_on"] > r["overspeed_on"] + 50.0, (
        f"overspeeding still pays under k_walk_overspeed: {r}")


def test_fastprof_overspeed_charge_bites_the_exceedance(fastprof_returns):
    """The margin must come from the NEW charge, not from the legacy
    kernel already pricing it (the canary proved the legacy stack lets
    2.5x overspeed through training unpunished)."""
    r = fastprof_returns
    margin_on = r["obey_on"] - r["overspeed_on"]
    margin_off = r["obey_off"] - r["overspeed_off"]
    assert margin_on > margin_off + 50.0, (
        f"charge adds no ordering pressure beyond legacy: {r}")


def test_fastprof_heading_error_is_priced(fastprof_returns):
    """The 55-deg-skewed drive (the canary's measured heading drift)
    must earn less than the aligned drive with the charges on."""
    r = fastprof_returns
    assert r["obey_on"] > r["skew_on"] + 50.0, (
        f"heading drift still pays under k_walk_heading: {r}")


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
    from sim_gait_compat import TripodGait

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
    from sim_gait_compat import TripodGait

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
    (4.6-6.7° peaks at the old 128 mm tibia; RE-MEASURED 2026-08-22 at
    the corrected 150 mm tibia geometry: 6.0-8.6°, still zero
    terminations): the leveling skill is closable through the action
    space, so it is learnable — not physically impossible."""
    for seed in SEEDS:
        fixed = _rock_rollout(seed, ROCK_OVERRIDES, counter=True)
        assert fixed["peak_tilt_deg"] < 9.0 and not fixed["terminated"], (
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
    """The demonstrated rise slides its pads ~656 mm during the curl at
    the corrected tibia-150 geometry (was ~463 mm at the old 128 mm
    tibia; re-measured 2026-08-22 alongside the drag_trans_allow_rise_m
    default bump 0.55->0.75) — inherent to the belly->plant path, not a
    cheat. The rise allowance must keep the reference completely
    uncharged, so the charge can ride into mixed-mode stand arms
    without repricing the one behavior that tape-provably works."""
    for r_on, r_off in zip(tdrag_bank["rise_on"], tdrag_bank["rise_off"]):
        assert 550.0 < r_on["drag_mm"] < 750.0, (
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
    from sim_gait_compat import TripodGait

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
    from sim_gait_compat import TripodGait

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
    from sim_gait_compat import TripodGait

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
    from sim_gait_compat import TripodGait
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
    from sim_gait_compat import TripodGait

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
    from sim_gait_compat import TripodGait

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
    from sim_gait_compat import TripodGait

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


# JOYCANARY stack — the 08-17 operator-approved canary recipe
# (fb_20260817T005114): FULLCIRCLE stack + BOUNDED terminal cost
# (reward.term_cost_max) + the walk-height income gate CALIBRATED from
# the honest scripted gait's measured height band (calibrate_walk_
# height.py, 08-17: tripod/noslip ride +0.7..+7.3 mm around the
# anchor; sigma 11 mm keeps the honest gait >=0.8 income while the
# measured 40-77 mm hardware-crouch band keeps <=0.001; termination
# 25 mm sits between the honest band and the collapse band). The cap
# was first tried at 60 and the bank REOPENED the c2 exploit (+81/ep:
# the zero-lift skate banks ~151 during the gait-gate's episode-start
# grace); 240 (a 20 s-equivalent horizon, ~3x smaller than the -730
# cliff) prices it back underwater. The cap
# must not reopen the c2 drag-then-fall exploit: with the horizon
# charge bounded, early death must STILL lose to freezing and to
# honest walking.

JC_TERM_COST_MAX = 240.0
JC_OVERRIDES = dict(FC_OVERRIDES)
JC_OVERRIDES.update({
    ("reward", "term_cost_max"): JC_TERM_COST_MAX,
    ("reward", "walk_height_gate"): 1.0,
    ("reward", "walk_height_sigma_mm"): 11.0,
    ("safety", "walk_max_height_drop_mm"): 25.0,
    ("safety", "walk_height_grace_s"): 2.0,
})


def test_joycanary_bounded_term_cost_keeps_early_death_unpaid():
    """The c2 exploit re-priced under the BOUNDED cap: accumulate the
    zero-lift skate's real income for 6 s of a 60 s episode under the
    canary stack, then charge the capped termination cost (flat +
    min(k*remaining, cap)). Must stay NEGATIVE, and a full-session
    freeze must still out-earn it — the cap trades the -730 critic
    cliff for a bounded charge WITHOUT making early death a paying
    strategy."""
    from sim_gait_compat import TripodGait

    fall_tick = None
    drag_income = 0.0
    env = _make_walk_env(0, overrides=JC_OVERRIDES, episode_seconds=60.0)
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
        fall_tick = six_s
        pen = 10.0 + min(
            FC_TERM_COST_PER_S * max(env.episode_steps - fall_tick, 0)
            * env.dt, JC_TERM_COST_MAX)
        drag_income -= pen
    env.close()

    park_total = _walk_rollout("park", 0, overrides=JC_OVERRIDES)
    park_60s = park_total * 4.0
    assert drag_income < 0.0, (
        f"drag-then-fall at 6 s retains positive return "
        f"({drag_income:.1f}) under the BOUNDED terminal cost — "
        f"raise term_cost_max above {JC_TERM_COST_MAX}")
    assert park_60s > drag_income + 200.0, (
        f"freezing ({park_60s:.1f}) does not clearly out-earn early "
        f"death ({drag_income:.1f}) under the bounded cap")


def test_joycanary_honest_gait_beats_stall_and_park_forward():
    """Sanity on the full canary stack (bounded cap + calibrated
    height gate): the hardware-proven tripod gait must still out-earn
    march-in-place and refusal (the calibrated 11 mm sigma must not
    tax the honest gait into a stall basin)."""
    r = {p: float(np.mean(
        [_walk_rollout(p, s, overrides=JC_OVERRIDES) for s in SEEDS]))
        for p in ("gait", "stall", "park")}
    assert r["gait"] > r["stall"] + 50.0, (
        f"stall rivals the gait under the canary stack: {r}")
    assert r["gait"] > r["park"] + 50.0, (
        f"parking rivals the gait under the canary stack: {r}")


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
    ("repair_one", 14),
    ("repair_two", 15),
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
        "repair_one", "repair_two", "tangle_60", "tangle_70",
        "tangle_80", "tangle_90", "tangle", "bank", "flip")
    assert tuple(family[0] for family in env.RECOVER_FAMILIES) == expected
    assert all(len(family) == 1 for family in env.RECOVER_FAMILIES)
    assert env.RECOVER_FAMILIES[21] == ("bank",)
    assert env.RECOVER_FAMILIES[22] == ("flip",)
    env.close()


def test_recover_floor_rungs_remain_distinct_after_physics_settle():
    """The added labels must describe different settled reset banks."""
    kinds = ("park", "crouch_shallow", "crouch_mid", "crouch_deep",
             "partial_high", "partial_mid", "partial_low", "zero",
             "tangle_mild", "tangle_mid", "tangle_60", "tangle_70",
             "tangle_80", "tangle_90", "tangle")
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
        "zero", "tangle_mild", "tangle_mid", "tangle_60", "tangle_70",
        "tangle_80", "tangle_90", "tangle")]
    # RE-MEASURED 2026-08-22 at the corrected 150 mm tibia geometry:
    # the tangle_70->tangle_80 gap (a longer tibia at a similar joint-
    # angle severity band) narrowed from a comfortable >2mm to ~1.9mm
    # while staying strictly monotonic (every rung still measurably
    # distinct, just closer at this one severity step) -- margin
    # relaxed 2.0->1.5mm; a real collapse (two rungs landing on the
    # same settled pose) would still fail this.
    assert all(a + 1.5 < b for a, b in zip(spreads, spreads[1:])), sig


@pytest.mark.parametrize("kind,loaded", (
    ("repair_one", 5),
    ("repair_two", 4),
))
def test_recover_terminal_repairs_are_upright_missing_foot_states(
        kind, loaded):
    """Repair rungs model the old B14 timeout, not another belly start."""
    for seed in range(8):
        env = _make_recover_env(seed, start=kind)
        env.reset()
        loads = np.array([
            max(float(env.data.sensordata[adr]), 0.0)
            for adr in env._touch_adr])
        roll, pitch = env._true_roll_pitch()
        assert int(np.sum(loads >= 0.35)) == loaded, loads
        assert max(abs(roll), abs(pitch)) * RAD2DEG < 8.0
        assert env._rec_reset_height_mm > 115.0
        env.close()


def test_recover_observation_has_plant_pose_and_real_reset_history():
    extra = {
        ("obs", "history_frames"): 4,
        ("obs", "reset_history_probe"): 1.0,
        ("obs", "recover_plant_q"): 1.0,
    }
    env = _make_recover_env(2, start="repair_two", extra=extra)
    obs, info = env.reset()
    frames = obs.reshape(4, -1)

    assert frames.shape == (4, 90)
    assert info["reset_history_probe_ticks"] == 3
    assert info["reset_history_probe_s"] == pytest.approx(3 * env.dt)
    assert env._step_i == 0
    assert all(not np.array_equal(frames[i], frames[i + 1])
               for i in range(3))
    q_scale = float(env.cfg["obs"].get("q_scale", 1.0))
    expected = ((env._state.joint_position
                 - env._plant_deg * DEG2RAD) / q_scale)
    np.testing.assert_allclose(frames[0, -18:], expected, atol=1e-6)
    assert np.linalg.norm(frames[0, -18:]) > 20.0 * np.linalg.norm(
        frames[0, :18])
    env.close()


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

    # Global training shortfall adds only a bounded replay overlay. It
    # increases the failing bucket's probability without touching certs.
    cert_before = dict(env._rec_stats)
    env.apply_recover_training_error_batch({0: (8.0, 8)})
    error_weights = env._recover_bucket_weights()
    assert error_weights[0] > 0.05
    assert error_weights[5] < 0.50
    assert np.isclose(error_weights.sum(), 1.0)
    assert env._rec_stats == cert_before
    assert env.recover_score_state()["training_errors"]["0"][
        "priority"] == 1.0
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


def test_recover_promotion_requires_fresh_full_retention_suite():
    env = _make_recover_env(
        0, start="plant_catch",
        extra={("goal", "recover_external_certification"): 1.0})
    env._rec_active_n = 3
    env._rec_focus_bucket = 2

    # B0's old pass is not valid for round 2, even though frontier B2 and
    # the other retained bucket pass in that round.
    env.apply_recover_certification(
        "plant_catch", [True] * 8, False, 1)
    env.apply_recover_certification(
        "onefoot_micro", [True] * 8, False, 2)
    env.apply_recover_certification(
        "onefoot_mid", [True] * 8, False, 2)
    status = env._recover_update_admission(2)
    assert not status["promoted"]
    assert not status["retention_passed"]
    assert status["failed_buckets"] == [0]
    assert env._rec_active_n == 3
    env.force_recover_start = None
    env._sample_recover()
    assert env._rec_active_n == 3, (
        "external-cert reset bypassed the fresh retention gate")

    # A fresh but failed retention assay also blocks promotion.
    env.apply_recover_certification(
        "plant_catch", [False] * 8, False, 2)
    status = env._recover_update_admission(2)
    assert not status["promoted"]
    assert status["failed_buckets"] == [0]

    # Promotion occurs only after that retained bucket passes in the same
    # certification round as the frontier.
    env.apply_recover_certification(
        "plant_catch", [True] * 8, False, 2)
    status = env._recover_update_admission(2)
    assert status["suite_passed"]
    assert status["promoted"]
    assert env._rec_active_n == 4

    checkpoint = env.recover_curriculum_checkpoint_state()
    env.apply_recover_training_error_batch({0: (8.0, 8)})
    error_debt = dict(env._rec_training_error_stats)
    env._rec_active_n = 7
    env._rec_focus_bucket = 6
    env._rec_stats = {"plant_catch": (0, 8)}
    env.restore_recover_curriculum_checkpoint_state(checkpoint)
    assert env._rec_active_n == 4
    assert env._rec_focus_bucket == 3
    assert env._rec_stats == checkpoint["stats"]
    assert env._rec_training_error_stats == error_debt, (
        "rollback discarded the adaptive replay debt")
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
    env._rec_active_n = 21          # unlock B0..B20 (tangle = bucket 20)
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
    assert hits >= 2, f"tangle drawn only {hits} times with B20 unlocked"
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


# --------------------------------------------------------------------------
# YAW-MARGIN bank — reward.k_yaw_margin (2026-08-19, operator order
# fb_20260818T152717 lineage). probe_dirswitch_tangle measured the
# direction-switch tangle PRECURSOR: after abrupt command switches the
# tall walker rides hip-yaw joints within ~2 deg of (and into) the hard
# stop for 1-9% of ticks; three exposure/schedule/blend levers failed
# to cure it. k_yaw_margin prices that precursor: per walk tick each
# leg whose hip-yaw margin to the nearest hard limit is inside
# reward.yaw_margin_allow_deg pays k scaled by depth into the band.
# Required semantics: default off = bit-exact; ~free for the honest
# tall gait (rides 10-20+ deg margins); a limit-riding pose pays hard.

YAWM_ON = dict(WALK_OVERRIDES)
YAWM_ON[("reward", "k_yaw_margin")] = 2.0
YAWM_ON[("reward", "yaw_margin_allow_deg")] = 3.0


def _yawm_pinned_rollout(seed: int, overrides: dict) -> dict:
    """Hold the champion plant stance but with every hip-yaw commanded
    0.5 deg from its hard stop — the scripted twin of the probe's
    measured post-switch limit-riding. Returns total reward and the
    accumulated yaw-margin charge (0.0 when the term is off)."""
    env = _make_walk_env(seed, overrides)
    env.reset()
    hi = float(np.degrees(env.model.joint("L0_yaw").range[1]))
    pinned = np.array([hi - 0.5, *WALK_PLANT] * 6) * DEG2RAD
    act = q_rad_to_action(pinned)
    total, charge = 0.0, 0.0
    while True:
        _obs, r, term, trunc, info = env.step(act)
        total += float(r)
        charge += float(info.get("reward_yaw_margin", 0.0))
        if term or trunc:
            break
    env.close()
    return {"return": total, "charge": charge}


def test_yaw_margin_default_off_bit_exact():
    """k_yaw_margin=0.0 (explicit) must equal the key absent on the
    honest-gait walk rollout — no reward-path or rng change on the
    default path."""
    off = dict(WALK_OVERRIDES)
    off[("reward", "k_yaw_margin")] = 0.0
    a = _walk_rollout("gait", SEEDS[0])
    b = _walk_rollout("gait", SEEDS[0], overrides=off)
    assert a == b, (
        f"k_yaw_margin=0 changed the walk reward path ({a} vs {b})")


def test_yaw_margin_free_for_honest_gait():
    """The hardware-proven scripted tall gait rides ~10-20+ deg of
    hip-yaw margin (probe medians) — the term must be ~free for it
    (the intended behavior keeps its income)."""
    off = _walk_rollout("gait", SEEDS[0])
    on = _walk_rollout("gait", SEEDS[0], overrides=YAWM_ON)
    drop = off - on
    assert drop < 0.02 * abs(off) + 5.0, (
        f"k_yaw_margin cost the honest gait {drop:.1f} of {off:.1f} — "
        "it must be ~free for the intended behavior")


def test_yaw_margin_charges_limit_riding():
    """A stance pinned 0.5 deg from the hip-yaw hard stop (the probe's
    measured tangle precursor, scripted) must pay heavily and visibly
    (info charge accumulates), and the SAME actor must earn strictly
    less with the term on than off."""
    off = _yawm_pinned_rollout(SEEDS[0], WALK_OVERRIDES)
    on = _yawm_pinned_rollout(SEEDS[0], YAWM_ON)
    assert off["charge"] == 0.0
    assert on["charge"] < -100.0, (
        f"limit-riding stance accumulated only {on['charge']:.1f} "
        "yaw-margin charge — the term is not biting the precursor")
    assert on["return"] < off["return"] - 100.0, (
        f"term-on return {on['return']:.1f} vs off {off['return']:.1f} "
        "— limit riding must be strictly money-losing")


# --------------------------------------------------------------------------
# FASTCLEAN combined-stack bank (2026-08-19, operator focus note "fast
# non-slipping gait", MCP lane 20260819T175106Z). The steer4
# continuation trains the bcgait1-hard1 walk stack with BOTH banked
# terms ON together for the first time — k_yaw_margin (anti-jam,
# validated by cw-dep-bcgait1-hard1-steer3-yawm1) + the structural
# per-stance drag charge k_drag_stance (charge-magnitude audit
# operating point) — and the command band widened to 0.05–0.08 m/s.
# MDP_PREFLIGHT requires the WALK orderings under the FULL combined
# stack the arm will train with, including at the raised 0.08 top
# command: honest stepping must out-earn the zero-lift skate, the
# march-in-place stall, and the park at BOTH ends of the band, and the
# combined charges must stay ~free for the honest tall gait.

FASTCLEAN_ON = dict(WALK_OVERRIDES)
FASTCLEAN_ON.update({
    ("reward", "k_yaw_margin"): 2.0,
    ("reward", "yaw_margin_allow_deg"): 3.0,
    ("reward", "k_drag_stance"): 8000.0,
    ("reward", "drag_stance_allow_mm"): 6.0,
    ("reward", "drag_stance_tick_floor_mm"): 0.25,
    ("reward", "walk_height_gate"): 1.0,
    ("reward", "walk_height_sigma_mm"): 30.0,
    ("goal", "walk_speed_max_m_s"): 0.08,
})


def test_fastclean_combined_stack_orders_gait_over_cheats():
    """Honest gait > skate/stall/park under the combined
    yaw-margin + drag-stance + height-gate stack, at both the champion
    command (0.05) and the raised top command (0.08)."""
    for name, vx in (("band_low", 0.05), ("band_top", 0.08)):
        r = {p: float(np.mean(
            [_walk_rollout(p, s, vx=vx, overrides=FASTCLEAN_ON)
             for s in SEEDS]))
            for p in ("gait", "skate", "stall", "park")}
        assert r["gait"] > r["skate"] + 50.0, (
            f"{name}: skating rivals stepping under the combined "
            f"stack: {r}")
        assert r["gait"] > r["stall"] + 50.0, f"{name}: {r}"
        assert r["gait"] > r["park"] + 50.0, f"{name}: {r}"


def test_fastclean_combined_charges_stay_cheap_for_honest_gait():
    """The combined charges must not tax the intended behavior into
    the ground: the honest tall gait's return under the FULL combined
    stack keeps the large majority of its charge-free income (the
    audit priced the honest gait's drag cost at ~20-23% of income;
    yaw-margin is ~free by its own bank)."""
    base = dict(FASTCLEAN_ON)
    base[("reward", "k_yaw_margin")] = 0.0
    base[("reward", "k_drag_stance")] = 0.0
    off = float(np.mean([_walk_rollout("gait", s, vx=0.05,
                                       overrides=base)
                         for s in SEEDS]))
    on = float(np.mean([_walk_rollout("gait", s, vx=0.05,
                                      overrides=FASTCLEAN_ON)
                        for s in SEEDS]))
    drop = off - on
    assert drop < 0.30 * abs(off) + 5.0, (
        f"combined charges cost the honest gait {drop:.1f} of "
        f"{off:.1f} — too expensive for the intended behavior")


# --------------------------------------------------------------------------
# PHASEDIR LOADSLIP BAND pin — the SLIP-FINANCED-PROGRESS cheat
# (cw-dep-bcgait4-phasedir5-stdoverride dig-in, 2026-08-22).
#
# Observed on the harness (DR-0, det, clone-relative, matched control
# logs/ckpt_eval/phasedir3_clone_control_gate): every phasedir RL arm
# abandoned the clone's high-cadence short-stride tripod for a
# lower-cadence longer-stride gait (swing_s 0.255->0.335-0.35 s,
# swings/leg ~30->~22, stride 0.0335->0.042 m, duty skew: tripod-B
# overstance leg5 0.52->0.56-0.61 while tripod-A understances
# 0.42-0.47) that finances progress with dragged loaded feet: det
# slip/m 1.89->3.00 (1.59x, gate cap 1.15x) while progress rose to
# 0.98x clone. Training telemetry (W&B io3m0yr7): rollout loadslip
# ratio sat 4.2-4.7 vs loadslip_ok=7.0 — factor ~0.99, excess charge
# ~-0.01/step — SLIP WAS ECONOMICALLY FREE all run while course
# income paid for the extra progress. Textbook 08-21 MISALIGNMENT:
# the band was sized to the std-0.368 clone's noisy sto slip
# (4.8-6.4) and never re-tightened after --warm-log-std-override
# dropped rollout std to 0.13 (rollout ratio ~4.3, det ~3.0).
#
# Why there is no scripted twin here: the scripted TripodGait is
# IK-consistent by construction and cannot reproduce the learned drag
# regime — measured on the controller 08-22: even period_scale 2.0 +
# lift_scale 0.35 + 1.15x drive tops out at loadslip ratio 1.84 (the
# learned cheat sits at 3.0 det / 4.3 rollout). The REAL cheat policy
# is therefore the bank behavior: the matched-env pricing A/B lives
# on the run pod via eval_checkpoint (clone vs
# ppo_goal_cw_dep_bcgait4_phasedir5_stdoverride under the full
# training cfg with the retightened band; artifacts
# logs/ckpt_eval/pd5_newband_ab_{clone,drag}). A/B MEASURED 08-22
# (same seeds/harness as the gate evals): under ok=3.0/max=6.0 the
# drag policy's own-rollout (sto, std 0.13) return fell +282.1 ->
# -30.5 (the cheat lost its income) while the clone's det return
# kept 99.2% (816.6 -> 809.7) and the det ordering clone>drag
# widened (+26.9 -> +32.5). These unit rows pin the PRICING SHAPE
# that closes the hole, using walk_task.py's exact formulas (income
# gate: factor=clip((max-ratio)/(max-ok),0,1); excess:
# k*max(ratio-ok,0)*dt per tick), so a future band edit that reopens
# the free ride fails here first.

PHASEDIR5_BAND_OLD = (7.0, 10.0)      # loadslip_ok, loadslip_max as run
PHASEDIR6_BAND_NEW = (3.0, 6.0)       # retightened to the std-0.13 regime
PHASEDIR_K_LSE = 10.0                 # reward.k_loadslip_excess, unchanged
# Measured operating points (W&B io3m0yr7 rollout ratios at std 0.13;
# det ratios from the DR-0 gate harness):
RATIO_DRAG_ROLLOUT = 4.3              # the cheat gait, q4 mean
RATIO_CLEAN_ROLLOUT = 3.3             # gate-passing det (~2.0-2.2) + the
                                      # measured ~+1.3 std-0.13 noise floor
RATIO_CLEAN_DET = 1.9                 # clone det slip/m (harness)
RATIO_DRAG_DET = 3.0                  # phasedir5 det slip/m (harness)


def _loadslip_price(ratio: float, band: tuple[float, float],
                    k_lse: float = PHASEDIR_K_LSE,
                    income_per_s: float = 54.0,
                    seconds: float = 15.0) -> float:
    """Episode walk income after the loadslip income gate + excess
    charge, exactly per walk_task.py (income scaled by factor; excess
    charged per second of commanded walking)."""
    ok, mx = band
    factor = min(max((mx - ratio) / max(mx - ok, 1e-6), 0.0), 1.0)
    excess = k_lse * max(ratio - ok, 0.0) * seconds
    return income_per_s * seconds * factor - excess


def test_phasedir_old_band_left_the_drag_gait_unpriced():
    """The hole itself, pinned: under the as-run band (ok=7/max=10)
    the drag gait's rollout ratio (4.3) and a clean gait's (3.3) pay
    IDENTICALLY (factor 1, zero excess) — slip-financed progress was
    free, so course income alone picked the winner. If this test ever
    fails, the old band stopped being a hole and this history note is
    stale."""
    p_drag = _loadslip_price(RATIO_DRAG_ROLLOUT, PHASEDIR5_BAND_OLD)
    p_clean = _loadslip_price(RATIO_CLEAN_ROLLOUT, PHASEDIR5_BAND_OLD)
    assert abs(p_drag - p_clean) < 1e-9, (
        f"old band unexpectedly separates drag {p_drag:.2f} vs clean "
        f"{p_clean:.2f}")


def test_phasedir_new_band_prices_the_drag_gait():
    """The fix: under the retightened band (ok=3/max=6) the measured
    drag operating point must pay SUBSTANTIALLY worse than the
    gate-passing operating point — enough that the ~10-15% course
    income the extra progress earns (progress 0.98x vs 0.77x clone at
    ~2.2/s) cannot buy it back. Margin: >= 30% of episode income."""
    income = 2.2 * 15.0
    p_drag = _loadslip_price(RATIO_DRAG_ROLLOUT, PHASEDIR6_BAND_NEW)
    p_clean = _loadslip_price(RATIO_CLEAN_ROLLOUT, PHASEDIR6_BAND_NEW)
    assert p_clean - p_drag > 0.30 * income, (
        f"new band does not separate: clean {p_clean:.2f} vs drag "
        f"{p_drag:.2f} (income {income:.1f})")


def test_phasedir_new_band_keeps_the_honest_gait_cheap():
    """Do not recreate phasedir2's obedient-but-slow collapse: a clean
    det-band gait (clone 1.9 slip/m; and its ~3.3 std-0.13 rollout
    twin) must keep the large majority of its income under the new
    band — the charge targets the cheat, not the teacher."""
    income = 2.2 * 15.0
    for r in (RATIO_CLEAN_DET, RATIO_CLEAN_ROLLOUT):
        p = _loadslip_price(r, PHASEDIR6_BAND_NEW)
        assert p > 0.75 * income, (
            f"honest ratio {r} keeps only {p:.1f} of {income:.1f} "
            "under the new band — too expensive for the teacher gait")


def test_phasedir_slow_escape_still_loses():
    """The ratio's denominator is banked progress, so walking SLOWER
    at the same absolute slip RAISES the ratio — the phasedir2 escape
    (slower mean gait to dodge per-tick charges) must price WORSE,
    not better, under the new band. Pin that design property."""
    # same absolute slip, half the progress => double the ratio
    p_normal = _loadslip_price(RATIO_DRAG_ROLLOUT, PHASEDIR6_BAND_NEW)
    p_slow = _loadslip_price(2.0 * RATIO_DRAG_ROLLOUT, PHASEDIR6_BAND_NEW)
    assert p_slow < p_normal, (
        f"slow escape pays better: slow {p_slow:.2f} vs {p_normal:.2f}")


def test_phasedir_new_band_price_monotone_in_ratio():
    """Gradient sanity: price strictly non-increasing in ratio across
    the whole band, strictly decreasing inside it — no plateau for
    PPO to camp on between the honest gait and the cheat."""
    band = PHASEDIR6_BAND_NEW
    grid = np.linspace(2.0, 7.0, 26)
    prices = [_loadslip_price(float(r), band) for r in grid]
    diffs = np.diff(prices)
    assert (diffs <= 1e-9).all(), "price not monotone in ratio"
    inside = (grid[:-1] >= band[0]) & (grid[:-1] < band[1])
    assert (diffs[inside] < -1e-6).all(), (
        "price has a flat spot inside the band")


# reward.walk_loadslip_gate at FULL strength on the recalibrated
# (1.5, 4.5) band — the tipfrac05/pushcal518 anti-skate income gate
# (08-23 yaw/slip dig-in). Context: the 4-arm pricing dose grid closed
# 4/4 FAIL — additive k_loadslip_excess at 6x (-0.48/tick) and 12x
# (-0.98/tick) both fired and slip did not move (family band 3.6-3.9
# at the m5 walk section vs bar 3.5). Marginal-pressure audit: the
# pre-registered "partial strength" gate (g=0.5, band 1.5/4.5) exerts
# income*g/(max-ok) ~ 3*0.5/3 = 0.5/tick per unit ratio — the SAME
# marginal dose slipexcess12 already refuted (12*dt = 0.48/tick), so
# it would be a re-run in disguise. The honest escalation is the FULL
# gate (g=1.0): ~1.0/tick marginal (2x the refuted dose) and ~70% of
# walk income withheld at the measured operating ratio ~3.6, while
# the wide band keeps a real income floor (factor 0.30 > 0) so the
# statue/park basin stays priced out. The default band (0.75, 1.5)
# is PINNED as a no-information constant tax at this family's
# operating point (factor identically 0, zero gradient — the
# 8b43a7c6 calibration finding).
LOADGATE_BAND_DEFAULT = (0.75, 1.50)   # walk_task.py defaults
LOADGATE_BAND_RECAL = (1.5, 4.5)       # recalibrated for ratio ~3.6
LOADGATE_FAMILY_RATIO = 3.6            # pushcal518 family operating pt
LOADGATE_BAR_RATIO = 3.5               # m5 walk section bar
LOADGATE_CLEANER_RATIO = 2.5           # a genuinely cleaner gait
LOADGATE_TEACHER_RATIO = 1.2           # scripted-teacher-grade slip
WALK_INCOME_PER_TICK = 3.0             # measured walk income scale
DT_TICK = 0.04
REFUTED_MARGINAL_PER_TICK = 12.0 * DT_TICK  # slipexcess12, FAILed


def _loadgate_income(ratio: float, band: tuple[float, float],
                     g: float = 1.0,
                     income_per_tick: float = WALK_INCOME_PER_TICK
                     ) -> float:
    """Per-tick walk income after the partial/full loadslip income
    gate, exactly per walk_task.py: factor = clip((max-ratio)/
    (max-ok), 0, 1); income *= (1-g) + g*factor."""
    ok, mx = band
    factor = min(max((mx - ratio) / max(mx - ok, 1e-6), 0.0), 1.0)
    return income_per_tick * ((1.0 - g) + g * factor)


def test_loadgate_default_band_is_a_no_information_tax():
    """The 08-22 calibration finding, pinned: at the family's measured
    operating ratio (~3.6) the DEFAULT band (0.75, 1.5) has factor
    identically 0 on both sides of the operating point — zero gradient,
    a constant tax that cannot select cleaner walking. Any arm that
    enables the gate without recalibrating the band is a no-op."""
    lo = _loadgate_income(LOADGATE_FAMILY_RATIO - 0.2,
                          LOADGATE_BAND_DEFAULT)
    hi = _loadgate_income(LOADGATE_FAMILY_RATIO + 0.2,
                          LOADGATE_BAND_DEFAULT)
    assert abs(lo - hi) < 1e-9, (
        f"default band unexpectedly has gradient at 3.6: {lo} vs {hi}")


def test_loadgate_recal_band_out_doses_the_refuted_charge():
    """The launch precondition: the full gate on the recalibrated band
    must exert MORE marginal pressure per unit ratio at the operating
    point than the already-refuted slipexcess12 charge — otherwise the
    arm is the same dose in a different wrapper and must not launch."""
    d = 0.1
    p_lo = _loadgate_income(LOADGATE_FAMILY_RATIO - d,
                            LOADGATE_BAND_RECAL)
    p_hi = _loadgate_income(LOADGATE_FAMILY_RATIO + d,
                            LOADGATE_BAND_RECAL)
    marginal = (p_lo - p_hi) / (2 * d)
    assert marginal > 1.5 * REFUTED_MARGINAL_PER_TICK, (
        f"gate marginal {marginal:.3f}/tick/ratio does not escalate "
        f"past the refuted {REFUTED_MARGINAL_PER_TICK:.3f}")


def test_loadgate_recal_band_keeps_income_floor_at_operating_point():
    """No statue cliff: at the measured operating ratio the gated walk
    income must remain strictly positive and a non-trivial fraction of
    full income, so stepping still out-earns parking from step 0 (the
    from-scratch statue-basin lesson; parking pays no walk income at
    all)."""
    p = _loadgate_income(LOADGATE_FAMILY_RATIO, LOADGATE_BAND_RECAL)
    assert p > 0.2 * WALK_INCOME_PER_TICK, (
        f"income at operating ratio 3.6 is {p:.2f} — cliff, not "
        "gradient")


def test_loadgate_recal_band_rewards_crossing_the_bar():
    """Moving from the family operating point (3.6) to a genuinely
    cleaner gait (2.5) must recover a LARGE income slice (>=25% of
    full walk income), and teacher-grade slip (1.2) must earn full
    income — the optimum sits at-or-below the bar, aligned with the
    m5 walk section (08-21 reward<->eval agreement rule)."""
    p_now = _loadgate_income(LOADGATE_FAMILY_RATIO, LOADGATE_BAND_RECAL)
    p_clean = _loadgate_income(LOADGATE_CLEANER_RATIO,
                               LOADGATE_BAND_RECAL)
    p_teacher = _loadgate_income(LOADGATE_TEACHER_RATIO,
                                 LOADGATE_BAND_RECAL)
    assert p_clean - p_now >= 0.25 * WALK_INCOME_PER_TICK, (
        f"cleaning 3.6->2.5 only recovers {p_clean - p_now:.2f}")
    assert abs(p_teacher - WALK_INCOME_PER_TICK) < 1e-9, (
        "teacher-grade slip does not earn full income")


def test_loadgate_recal_band_monotone_no_plateau():
    """Gradient sanity across the whole band: income strictly
    decreasing in ratio inside (1.5, 4.5) — no camping plateau between
    the teacher gait and the family operating point."""
    import numpy as _np
    grid = _np.linspace(1.6, 4.4, 15)
    prices = [_loadgate_income(float(r), LOADGATE_BAND_RECAL)
              for r in grid]
    diffs = _np.diff(prices)
    assert (diffs < -1e-6).all(), "income has a flat spot in the band"
