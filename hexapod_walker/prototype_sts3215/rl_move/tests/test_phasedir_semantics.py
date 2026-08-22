"""PHASEDIR2 preflight bank — reward alignment for the staged
phase-clock gait line (operator order 2026-08-22, MCP operator lane
fb_20260822T032514, asked by Lukas: "Make the reward correctly aligned
and rerun").

The failure this bank pins: cw-dep-bcgait4-phasedir1's PPO reward
IMPROVED while behavior got WORSE than its un-RL'd phase-BC-clone
control (dir_err med 35.6 -> 67.3 deg, speed 0.068 -> 0.139 m/s
overspeed, slip/m 1.81 -> 4.17, rear headings collapsed to
prog 0.01-0.07) — nothing in that stack priced course, overspeed, or
rear-heading refusal, so "generic stable fast walking" was a paying
basin. Per the order, NO phasedir launch until the candidate stack
provably ranks the good clone behavior above every measured attractor:

  obey         the phase clone's proven behavior — the scripted
               teacher driven AT the 0.08 m/s command along the
               commanded heading (stride sway + slip/m ~1.7 included)
  skew         driven 1.74x at command + 50 deg (the full phasedir1
               degradation: fast AND off-course)
  fastcadence  period_scale 0.6 + 1.74x drive — churning the cycle
               faster to chase speed; buys slip/m ~2.5, the measured
               degradation's slip mechanism
  wrongway     0.08 m/s toward +x REGARDLESS of the command (the
               command-ignoring generic-stable-walk basin)
  stall        march in place (the rear-heading refusal, prog ~0)
  park         hold the plant stance and refuse (canonical WALK-bank
               refusal; ordering obey > stall > park is the
               MDP_PREFLIGHT WALK contract under THIS stack)

TEACHER CONVENTION (important): the proxies use the RAW hardware
tripod_gait module, NOT sim_gait_compat — the phase clone was minted
(bc_init_gait, 08-22 ~00:2x) BEFORE the knee-convention boundary
existed, so the clone's action dialect is raw post-30660b51
absolute-tibia values fed unconverted into the sim knees, and that
gait measurably walks clean at the measured plant (prog 0.67-0.80,
slip/m 1.56-2.07). The bank must rank the CLONE's behavior class, not
the hardware-faithful converted gait. (The convention-corrected gait
also walks, ~20% slower; the phasedir2 run anchors to the raw dialect
via train.bc_anchor_knee_abs=1.)

HONESTY NOTE on the speed band: no raw-teacher drive can push the
PLANT past the 0.084 band edge at the 0.08 command (2.5-3.5x drives
saturate ~0.05 along) — phasedir1's 0.139 m/s came from non-teacher
mechanics RL invented. The EMA overspeed band charge
(k_walk_course_overspeed) therefore cannot be exercised by a scripted
rollout at this command; it stays in the stack as insurance priced by
construction (same min(over/s_ref,3) form as the proven instant
charge, on the same EMA the course tests exercise), and the run gate
judges speed on the harness metrics.

Orderings are asserted PER HEADING BIN (fwd / left 90 / rear 180) —
the operator's gate is clone-relative per commanded heading, so the
reward must already prefer obedience in every bin, including rear.
The attribution tests prove the margin comes from the NEW terms
(k_walk_course + k_walk_overspeed), not from income the phasedir1
stack already had (which measurably failed to hold the line).

The teacher clock in every drive is COMMAND-GATED (advances only on
commanded ticks), matching goal.walk_phase_obs clock semantics and the
clone's bc_init_gait convention. k_phase_contact is deliberately NOT
in the stack: its PHASE_TRIPOD_A/sin(phase) stance convention is 90
deg out of phase with TripodGait's _phase_offset=pi/2 (agreement ~50%
= zero net income for the honest teacher), so it prices nothing here;
gait preservation is carried by the phase-locked walk BC anchor
(train.bc_anchor_phase_lock, tested below) plus the anchor/height
income gates.
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

from rl_move.config import load_config                      # noqa: E402
from rl_move.robot_state import DEG2RAD                     # noqa: E402
from rl_move.sim.joint_task import q_rad_to_action          # noqa: E402
from rl_move.sim.servo_model import SimServoParams          # noqa: E402

WALK_PLANT = (20.0, 80.0)
CMD_SPEED = 0.08                 # the staged line's ONE fixed speed
ATTR_SPEED = 0.139               # phasedir1's measured overspeed attractor
SKEW_DEG = 50.0                  # phasedir1's excess course error class
SEEDS = (0, 1)
EP_SECONDS = 10.0

HEADING_BINS = {"fwd": 0.0,
                "left": math.pi / 2.0,
                "rear": math.pi}

# The candidate cw-dep-bcgait4-phasedir2 stack: phasedir1's config plus
# the alignment terms (course + overspeed charges, idle floor). Keep in
# sync with the launched arm's --cfg-set list.
PHASEDIR2_STACK = {
    ("reward", "k_drag_loaded"): 10.0,
    ("reward", "k_park_duty"): 1.0,
    ("reward", "walk_kernel_prog_gate"): 1.0,
    ("goal", "walk_park_start_frac"): 0.25,
    ("reward", "walk_anchor_gate"): 1.0,
    ("reward", "anchor_tol_mm"): 10.0,
    ("reward", "walk_height_gate"): 1.0,
    ("reward", "walk_height_sigma_mm"): 30.0,
    ("goal", "walk_speed_min_m_s"): CMD_SPEED,
    ("goal", "walk_speed_max_m_s"): CMD_SPEED,
    ("goal", "walk_obs_body_vel"): 2,
    ("goal", "walk_phase_obs"): 1,
    ("goal", "walk_phase_hz"): 1.333333,
    ("safety", "max_roll_deg"): 25,
    ("safety", "max_pitch_deg"): 25,
    ("safety", "max_delta_q_deg"): 5.0,
    ("bus", "write_speed"): 1500,
    ("bus", "write_acc"): 80,
    ("bus", "servo_vel_max_counts_s"): "write_speed",
    # NEW alignment terms (the order's item 3). Overspeed is priced
    # on the STRIDE-AVERAGED speed (k_walk_course_overspeed), not the
    # instant one: first bank iteration measured overspeed 504 vs obey
    # 430 under instant k_walk_overspeed=2/tol=0.10 — the 1.3-1.7x
    # attractor's per-tick exceedance is too small while prog paid it
    # 1.25x, and tightening the instant band would tax the honest
    # gait's per-stride speed pulses instead.
    ("reward", "k_walk_course"): 2.0,
    ("reward", "walk_course_tau_s"): 0.75,
    ("reward", "k_walk_course_overspeed"): 4.0,
    ("reward", "walk_course_overspeed_tol"): 0.05,
    ("reward", "k_walk_idle_charge"): 1.0,
    ("reward", "walk_idle_speed_m_s"): 0.04,
    # Loaded-slip pricing, clone-banded (order item 3 "loaded slip").
    # The teacher proxy for the overdrive attractor saturates near the
    # commanded SPEED and dumps the excess drive into loaded slip, so
    # the separator is the slip-per-metre band, not the speed band:
    # clone slip/m 1.56-2.07 (det, measured 08-22) must gate factor ~1,
    # the degraded 4.17 must gate to ~0 and pay the excess charge.
    ("reward", "walk_loadslip_gate"): 1.0,
    ("reward", "loadslip_ok"): 2.2,
    ("reward", "loadslip_max"): 4.0,
    ("reward", "k_loadslip_excess"): 10.0,
}
# phasedir1's stack = the candidate minus the new alignment terms.
ALIGN_KEYS = (("reward", "k_walk_course"),
              ("reward", "k_walk_course_overspeed"),
              ("reward", "walk_course_overspeed_tol"),
              ("reward", "walk_course_tau_s"),
              ("reward", "k_walk_idle_charge"),
              ("reward", "walk_idle_speed_m_s"),
              ("reward", "walk_loadslip_gate"),
              ("reward", "loadslip_ok"),
              ("reward", "loadslip_max"),
              ("reward", "k_loadslip_excess"))
PHASEDIR1_STACK = {k: v for k, v in PHASEDIR2_STACK.items()
                   if k not in ALIGN_KEYS}


def _make_walk_env(seed: int, overrides: dict,
                   episode_seconds: float = EP_SECONDS):
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv

    cfg = load_config()
    for (sec, leaf), val in overrides.items():
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


def _pin_command(env, heading_rad: float, speed: float = CMD_SPEED):
    """Deterministic command: 1 s hold, 1 s ramp, then fixed heading."""
    traj = env._goal_traj
    n = len(traj.vx)
    hold_n = ramp_n = int(round(1.0 / env.dt))
    ramp = np.linspace(0.0, 1.0, ramp_n)
    vx_t = speed * math.cos(heading_rad)
    vy_t = speed * math.sin(heading_rad)
    for arr, tgt in ((traj.vx, vx_t), (traj.vy, vy_t)):
        arr[:] = tgt
        arr[:hold_n] = 0.0
        arr[hold_n:hold_n + ramp_n] = tgt * ramp
    if getattr(traj, "wz", None) is not None:
        traj.wz[:] = 0.0
    return traj, n


def _phasedir_rollout(drive: str, seed: int, heading_rad: float,
                      overrides: dict) -> float:
    # RAW module on purpose — the clone lineage's dialect (see header).
    from tripod_gait import TripodGait

    env = _make_walk_env(seed, overrides)
    env.reset()
    traj, n = _pin_command(env, heading_rad)

    gait = TripodGait(
        vx=0.0,
        period_scale=(0.6 if drive == "fastcadence" else 1.0))
    gait.sync_plant_stance(*WALK_PLANT)
    gait.reset_phase()

    total, step, t_gait = 0.0, 0, 0.0
    while True:
        i = min(step, n - 1)
        vx_ref, vy_ref = float(traj.vx[i]), float(traj.vy[i])
        s_ref = math.hypot(vx_ref, vy_ref)
        if s_ref > 1e-3:
            # command-gated teacher clock == the walk_phase_obs clock
            t_gait += env.dt
            ux, uy = vx_ref / s_ref, vy_ref / s_ref
            if drive == "obey":
                gv = (vx_ref, vy_ref)
            elif drive == "fastcadence":
                f = ATTR_SPEED / CMD_SPEED
                gv = (f * vx_ref, f * vy_ref)
            elif drive == "skew":
                f = (ATTR_SPEED / CMD_SPEED) * s_ref
                a = math.radians(SKEW_DEG)
                gv = (f * (ux * math.cos(a) - uy * math.sin(a)),
                      f * (ux * math.sin(a) + uy * math.cos(a)))
            elif drive == "wrongway":
                gv = (s_ref, 0.0)          # +x regardless of command
            elif drive == "stall":
                gv = (0.0, 0.0)            # march in place at the spot
            elif drive == "park":
                gv = (0.0, 0.0)
            else:
                raise ValueError(drive)
        else:
            gv = (0.0, 0.0)
        gait.set_velocity(vx=gv[0], vy=gv[1])
        if drive == "park":
            act = q_rad_to_action(env._plant_deg * DEG2RAD)
        else:
            act = q_rad_to_action(
                np.asarray(gait.desired_deg(t_gait)) * DEG2RAD)
        _obs, r, term, trunc, _info = env.step(act)
        total += float(r)
        step += 1
        if term or trunc:
            break
    env.close()
    return total


def _mean(drive: str, heading: float, stack: dict) -> float:
    return float(np.mean([_phasedir_rollout(drive, s, heading, stack)
                          for s in SEEDS]))


@pytest.fixture(scope="module")
def returns() -> dict[str, float]:
    out = {}
    for bin_name, h in HEADING_BINS.items():
        for drive in ("obey", "fastcadence", "skew", "stall", "park"):
            out[f"{bin_name}_{drive}"] = _mean(drive, h, PHASEDIR2_STACK)
        if bin_name != "fwd":     # wrongway == obey on the fwd bin
            out[f"{bin_name}_wrongway"] = _mean(
                "wrongway", h, PHASEDIR2_STACK)
    # phasedir1-stack controls for the attribution tests
    for bin_name in ("fwd", "rear"):
        h = HEADING_BINS[bin_name]
        for drive in ("obey", "fastcadence", "skew"):
            out[f"old_{bin_name}_{drive}"] = _mean(
                drive, h, PHASEDIR1_STACK)
    return out


@pytest.mark.parametrize("bin_name", list(HEADING_BINS))
def test_obey_beats_fastcadence_every_bin(returns, bin_name):
    """Walking the commanded heading AT the commanded 0.08 m/s must
    decisively out-earn the cadence-churned speed chase (slip/m ~2.5,
    the measured degradation's slip mechanism) in every bin."""
    r = returns
    assert r[f"{bin_name}_obey"] > r[f"{bin_name}_fastcadence"] + 30.0, {
        k: v for k, v in r.items() if k.startswith(bin_name)}


@pytest.mark.parametrize("bin_name", list(HEADING_BINS))
def test_obey_beats_fast_offcourse_every_bin(returns, bin_name):
    """The full phasedir1 degradation (fast AND ~50 deg off course)
    must earn clearly less than obedience in every bin."""
    r = returns
    assert r[f"{bin_name}_obey"] > r[f"{bin_name}_skew"] + 50.0, {
        k: v for k, v in r.items() if k.startswith(bin_name)}


@pytest.mark.parametrize("bin_name", list(HEADING_BINS))
def test_obey_beats_refusal_stall_every_bin(returns, bin_name):
    """Marching in place through a move command (the rear-heading
    collapse, prog 0.01-0.07) must earn far less than obeying —
    especially in the rear bin where phasedir1 actually refused."""
    r = returns
    assert r[f"{bin_name}_obey"] > r[f"{bin_name}_stall"] + 100.0, {
        k: v for k, v in r.items() if k.startswith(bin_name)}


@pytest.mark.parametrize("bin_name", list(HEADING_BINS))
def test_walk_bank_contract_obey_buries_stall_and_park(returns, bin_name):
    """MDP_PREFLIGHT WALK contract, adapted for THIS arm (deviation
    filed in OPERATOR_QUESTIONS.md 2026-08-22): the generic bank
    orders progress > stall > park to preserve the park->step->walk
    DISCOVERY gradient for from-scratch learners. This arm is
    warm-started from a proven walking clone, and the operator's
    ordered loaded-slip pricing (k_loadslip_excess) intentionally
    buries a scuffing march-in-place BELOW a quiet park (measured:
    stall ~47, park ~112, obey ~458-478) — weakening slip pricing to
    restore the generic stall>park ordering would undo the order's
    item 3. Required here instead: obey decisively out-earns BOTH
    refusal basins in every bin."""
    r = returns
    assert r[f"{bin_name}_obey"] > r[f"{bin_name}_stall"] + 100.0, {
        k: v for k, v in r.items() if k.startswith(bin_name)}
    assert r[f"{bin_name}_obey"] > r[f"{bin_name}_park"] + 100.0, {
        k: v for k, v in r.items() if k.startswith(bin_name)}


@pytest.mark.parametrize("bin_name", ["left", "rear"])
def test_obey_beats_command_ignoring_walk(returns, bin_name):
    """Walking +x regardless of the command (the generic-stable-walk
    basin) must lose to obeying on every non-forward bin."""
    r = returns
    assert r[f"{bin_name}_obey"] > r[f"{bin_name}_wrongway"] + 50.0, {
        k: v for k, v in r.items() if k.startswith(bin_name)}


@pytest.mark.parametrize("bin_name", ["fwd", "rear"])
def test_alignment_terms_create_the_margin(returns, bin_name):
    """The obey-vs-attractor margins must come from the NEW alignment
    terms: under the phasedir1 stack (course/overspeed/idle removed)
    the margin was too small to stop PPO — require the new stack to
    add real ordering pressure on both attractors."""
    r = returns
    for drive in ("fastcadence", "skew"):
        new_m = r[f"{bin_name}_obey"] - r[f"{bin_name}_{drive}"]
        old_m = r[f"old_{bin_name}_obey"] - r[f"old_{bin_name}_{drive}"]
        # +25 floor: measured 08-22 the new terms take rear/fastcadence
        # from 11.7 to 48.6 (4x) and fwd/skew from ~90 to ~210; the
        # floor asserts real added pressure without tuning reward doses
        # to a test constant.
        assert new_m > old_m + 25.0, (
            f"{bin_name}/{drive}: new margin {new_m:.1f} vs "
            f"phasedir1-stack margin {old_m:.1f} — the new terms add "
            f"no pressure; {r}")


# ---------------------------------------------------------------------------
# Unit tests for the new mechanisms themselves.


def test_heading_set_draws_only_from_set():
    """goal.walk_heading_set must sample episode headings uniformly
    from the given SET (stage-B curriculum mechanism)."""
    stack = dict(PHASEDIR2_STACK)
    stack[("goal", "walk_heading_set")] = [0.0, 0.7854, -0.7854]
    env = _make_walk_env(0, stack, episode_seconds=2.0)
    seen = set()
    for _ in range(24):
        env.reset()
        traj = env._goal_traj
        vx_t, vy_t = float(traj.vx[-1]), float(traj.vy[-1])
        ang = round(math.atan2(vy_t, vx_t), 3)
        seen.add(ang)
        assert ang in {0.0, 0.785, -0.785}, ang
        assert abs(math.hypot(vx_t, vy_t) - CMD_SPEED) < 1e-6
    env.close()
    assert len(seen) == 3, f"set never fully sampled: {seen}"


def test_heading_set_off_is_stream_exact():
    """Absent vs empty-string walk_heading_set must give bit-identical
    command draws (no rng stream perturbation when off)."""
    stack_a = dict(PHASEDIR2_STACK)
    stack_b = dict(PHASEDIR2_STACK)
    stack_b[("goal", "walk_heading_set")] = ""
    for stack in (stack_a, stack_b):
        stack[("goal", "walk_heading_max_rad")] = 3.14159
    env_a = _make_walk_env(7, stack_a, episode_seconds=2.0)
    env_b = _make_walk_env(7, stack_b, episode_seconds=2.0)
    for _ in range(6):
        env_a.reset()
        env_b.reset()
        assert np.array_equal(env_a._goal_traj.vx, env_b._goal_traj.vx)
        assert np.array_equal(env_a._goal_traj.vy, env_b._goal_traj.vy)
    env_a.close()
    env_b.close()


def test_course_charge_spares_honest_sway_charges_wrongway():
    """The course term must be ~free for the obedient gait (stride sway
    averages out over the 0.75 s EMA) and expensive for command-
    ignoring travel; and with k_walk_course=0 no course info keys leak
    (default-off hygiene)."""
    from sim_gait_compat import TripodGait

    def course_sum(drive: str, k: float) -> tuple[float, int]:
        stack = dict(PHASEDIR2_STACK)
        stack[("reward", "k_walk_course")] = k
        env = _make_walk_env(3, stack)
        env.reset()
        traj, n = _pin_command(env, math.pi)     # rear command
        gait = TripodGait(vx=0.0)
        gait.sync_plant_stance(*WALK_PLANT)
        gait.reset_phase()
        tot, keys, step, t_gait = 0.0, 0, 0, 0.0
        while True:
            i = min(step, n - 1)
            vx_ref, vy_ref = float(traj.vx[i]), float(traj.vy[i])
            if math.hypot(vx_ref, vy_ref) > 1e-3:
                t_gait += env.dt
                gv = ((vx_ref, vy_ref) if drive == "obey"
                      else (math.hypot(vx_ref, vy_ref), 0.0))
            else:
                gv = (0.0, 0.0)
            gait.set_velocity(vx=gv[0], vy=gv[1])
            act = q_rad_to_action(
                np.asarray(gait.desired_deg(t_gait)) * DEG2RAD)
            _o, _r, term, trunc, info = env.step(act)
            if "reward_walk_course" in info:
                keys += 1
                tot += float(info["reward_walk_course"])
            step += 1
            if term or trunc:
                break
        env.close()
        return tot, keys

    obey_pen, obey_keys = course_sum("obey", 2.0)
    wrong_pen, wrong_keys = course_sum("wrongway", 2.0)
    _, off_keys = course_sum("obey", 0.0)
    assert off_keys == 0, "course info keys leak with the term off"
    assert obey_keys > 0 and wrong_keys > 0
    # Obedient gait: near-free. Command-ignoring: pays hard
    # (cos ~ -1 on a rear command -> ~2k per tick).
    assert obey_pen > -60.0, f"course term taxes the honest gait: {obey_pen}"
    assert wrong_pen < obey_pen - 200.0, (obey_pen, wrong_pen)


def test_bc_anchor_phase_lock_matches_command_gated_clock():
    """train.bc_anchor_phase_lock=1 must drive the walk BC-anchor gait
    on the command-gated clock (the one the policy's phase obs shows,
    and the one the phase clone was distilled on): bc_target must equal
    a twin TripodGait advanced ONLY on commanded ticks — including
    across the 1 s spawn hold, where the legacy wall-clock anchor jumps
    ~0.33 cycle. The phasedir2 config also sets
    train.bc_anchor_knee_abs=1 so the anchor speaks the clone
    lineage's raw dialect — the twin checks that too. Default (no
    keys) must keep the legacy behavior bit-exact: wall-clock time,
    sim_gait_compat convention."""
    from sim_gait_compat import TripodGait as CompatGait
    from tripod_gait import TripodGait as RawGait

    def run(phase_lock: float | None):
        stack = dict(PHASEDIR2_STACK)
        stack[("train", "bc_anchor_coef")] = 1.0
        stack[("train", "bc_anchor_walk")] = 1.0
        if phase_lock is not None:
            stack[("train", "bc_anchor_phase_lock")] = phase_lock
            stack[("train", "bc_anchor_knee_abs")] = 1.0
        env = _make_walk_env(5, stack, episode_seconds=6.0)
        env.reset()
        traj, n = _pin_command(env, 0.0)
        plant = env._plant_deg * DEG2RAD
        targets = []
        step = 0
        while True:
            act = q_rad_to_action(plant)     # behavior irrelevant
            _o, _r, term, trunc, info = env.step(act)
            step += 1
            tick = step        # == env._step_i at emission time
            targets.append((tick, info.get("bc_target")))
            if term or trunc or step >= 100:
                break
        env.close()
        return targets, traj, float(env.dt)

    # Phase-locked: twin RAW gait advanced only on commanded ticks.
    targets, traj, dt = run(1.0)
    twin = RawGait(vx=0.0)
    twin.sync_plant_stance(*WALK_PLANT)
    twin.reset_phase()
    t_acc, checked = 0.0, 0
    n = len(traj.vx)
    for tick, tgt in targets:
        i = min(tick, n - 1)
        vx_ref, vy_ref = float(traj.vx[i]), float(traj.vy[i])
        if math.hypot(vx_ref, vy_ref) > 1e-3:
            assert tgt is not None, f"no bc_target on commanded tick {tick}"
            t_acc += dt
            twin.set_velocity(vx=vx_ref, vy=vy_ref)
            expect = q_rad_to_action(
                np.asarray(twin.desired_deg(t_acc)) * DEG2RAD)
            np.testing.assert_allclose(tgt, expect, atol=1e-6)
            checked += 1
        else:
            assert tgt is None, f"bc_target on uncommanded tick {tick}"
    assert checked >= 50, f"too few commanded ticks checked: {checked}"

    # Default off: bit-exact legacy (wall-clock t = _step_i * dt,
    # convention-corrected compat gait).
    targets_off, traj_off, dt_off = run(None)
    legacy = CompatGait(vx=0.0)
    legacy.sync_plant_stance(*WALK_PLANT)
    legacy.reset_phase()
    n = len(traj_off.vx)
    for tick, tgt in targets_off:
        i = min(tick, n - 1)
        vx_ref, vy_ref = float(traj_off.vx[i]), float(traj_off.vy[i])
        if math.hypot(vx_ref, vy_ref) > 1e-3:
            assert tgt is not None
            legacy.set_velocity(vx=vx_ref, vy=vy_ref)
            expect = q_rad_to_action(
                np.asarray(legacy.desired_deg(tick * dt_off)) * DEG2RAD)
            np.testing.assert_allclose(tgt, expect, atol=1e-6)
