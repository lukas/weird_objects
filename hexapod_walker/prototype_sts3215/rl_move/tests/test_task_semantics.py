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
# Banks still owed (binding: build the bank BEFORE the next arm of that
# mode launches; a skipped test here is a launch blocker for that mode).


@pytest.mark.skip(reason="LOWER bank not yet recorded — needs: honest "
                         "belly-down lower (pads legitimately 20-45 mm "
                         "up) > partial descent > full-height refusal > "
                         "the outrigger cheat (cw-stance-postgate1) / "
                         "one-leg-aloft cheat (cw-stand-b2p1) > unsafe. "
                         "BUILD BEFORE the next lower arm.")
def test_lower_correct_beats_outrigger_cheat():
    raise NotImplementedError


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
})
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
                  overrides: dict | None = None) -> float:
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

    gait = TripodGait(vx=0.0)
    gait.sync_plant_stance(*WALK_PLANT)
    plant_rad = np.array([0.0, *WALK_PLANT] * 6) * DEG2RAD
    gait.reset_phase()

    total, step = 0.0, 0
    while True:
        t = step * env.dt
        i = min(step, n - 1)
        if policy == "gait":
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


def test_omni_turn_ordering_survives_repricing():
    """The TURN bank ordering (turn > partial > drift > park) must hold
    under the omni stack too (k_current=0 + dep contract could in
    principle flip a margin)."""
    t = {p: float(np.mean([_turn_rollout(p, s_wz * TURN_CMD_WZ, s,
                                         overrides=OMNI_OVERRIDES)
                           for s in SEEDS for s_wz in (+1.0, -1.0)]))
         for p in ("turn", "partial", "drift", "park")}
    assert t["turn"] > t["partial"] > t["drift"] > t["park"], t
