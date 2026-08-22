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

PHASEDIR3 REPRICE (2026-08-22, operator focus fb 20260822T051709Z):
after cw-dep-bcgait4-phasedir2-staged-fwd failed obedient-but-slow
(progress 0.836x clone; the det-band loadslip/overspeed charges taxed
PPO's OWN exploration noise at std 0.36 and the cheapest gradient was
a shrunken gait), the stack below is repriced for the TRAINING
regime: loadslip thresholds sit above the measured noisy-clone band
(ok 7.0 / max 10.0), overspeed uses the unbiased along-command
projection (walk_course_overspeed_along=1), and the course charge is
floored at 0.04 m/s smoothed speed. The NOISY-REGIME tests at the
bottom of this file pin all of it; the det-band anti-skate claims the
loadslip section used to make were RETIRED with measurement evidence
(see the stack comment) — det-band slip is owned by the eval gate +
the phase-locked BC anchor.

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
    # Loaded-slip pricing, REPRICED to the measured NOISY-clone band
    # (2026-08-22 phasedir3, operator focus fb 20260822T051709Z).
    # phasedir2-staged-fwd showed the det-band thresholds (ok 2.2 /
    # max 4.0) tax stochastic exploration itself: the clone driven at
    # PPO's stuck std=0.36 measures ratio 4.8-6.4 (fwd 5.91, rear
    # 4.76, stall 6.41) because exploration jitter is REAL loaded
    # slip — the whole run trained at income factor ~0.07 plus a flat
    # -1.19/tick excess charge, and the cheapest gradient was a
    # shrunken gait (progress 0.836x clone). Filtering was REFUTED by
    # measurement, not assumed away: an EMA-position slip twin
    # (tau 0.08-0.4 sweep) attenuates the honest scuff signal FASTER
    # than the noise (raw S/N 0.69/4.2 vs 0.31/2.89 at the best tau)
    # because both live in the same stride-frequency band. So the
    # thresholds move above the noisy-clone band: gross skating
    # (ratio > 7) still gates income to 0 by 10 and pays the excess
    # charge; det-band slip (clone 1.7 vs degraded 4.17) is no longer
    # priced in TRAINING and is owned by the eval gate (slip <= 1.15x
    # clone) + the phase-locked BC anchor (which preserved the gait
    # 100% in phasedir2).
    ("reward", "walk_loadslip_gate"): 1.0,
    ("reward", "loadslip_ok"): 7.0,
    ("reward", "loadslip_max"): 10.0,
    ("reward", "k_loadslip_excess"): 10.0,
    # phasedir3 reprice (same order): overspeed priced on the UNBIASED
    # along-command projection of the course EMA — |v_ema| is biased
    # upward by zero-mean sway (norm of a noisy vector) and fired flat
    # ~-3/tick on the noisy clone whose along travel was UNDER the
    # band. Course charge gated to smoothed speeds >= 0.04 m/s (half
    # command): below that the EMA direction is noise, and directed
    # wrong-way travel at command speed still clears the bar and pays.
    ("reward", "walk_course_overspeed_along"): 1.0,
    ("reward", "walk_course_min_speed_m_s"): 0.04,
}
# phasedir1's stack = the candidate minus the new alignment terms.
ALIGN_KEYS = (("reward", "k_walk_course"),
              ("reward", "k_walk_course_overspeed"),
              ("reward", "walk_course_overspeed_tol"),
              ("reward", "walk_course_overspeed_along"),
              ("reward", "walk_course_min_speed_m_s"),
              ("reward", "walk_course_tau_s"),
              ("reward", "k_walk_idle_charge"),
              ("reward", "walk_idle_speed_m_s"),
              ("reward", "walk_loadslip_gate"),
              ("reward", "loadslip_ok"),
              ("reward", "loadslip_max"),
              ("reward", "k_loadslip_excess"))
NOISE_STD = 0.36                 # phasedir2's measured stuck train std
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
                      overrides: dict, noise_std: float = 0.0,
                      collect: dict | None = None) -> float:
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

    rng = np.random.default_rng(1000 + seed)
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
            elif drive == "shrunk":
                # phasedir2's measured failure basin: same command,
                # gait driven at 0.75x of it (progress 0.836x clone,
                # speed at the 0.060 band floor).
                gv = (0.75 * vx_ref, 0.75 * vy_ref)
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
        if noise_std > 0.0:
            # PPO's exploration regime: Gaussian action noise at the
            # measured stuck std, clipped to the action space (SB3
            # clips sampled actions the same way).
            act = np.clip(act + rng.normal(0.0, noise_std, act.shape),
                          -1.0, 1.0)
        _obs, r, term, trunc, info = env.step(act)
        total += float(r)
        step += 1
        if collect is not None:
            for k in ("reward_loadslip_excess",
                      "reward_walk_course_overspeed",
                      "reward_walk_course", "reward_drag_stance",
                      "reward_walk", "reward_walk_prog"):
                collect[k] = collect.get(k, 0.0) + float(info.get(k, 0.0))
            for k in ("walk_loadslip_ratio", "walk_loadslip_factor"):
                if k in info:
                    collect[k] = float(info[k])
        if term or trunc:
            break
    env.close()
    return total


def _mean(drive: str, heading: float, stack: dict,
          noise_std: float = 0.0) -> float:
    return float(np.mean([_phasedir_rollout(drive, s, heading, stack,
                                            noise_std=noise_std)
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
    out-earn the cadence-churned speed chase in every bin. Bar reduced
    +30 -> +8 by the phasedir3 reprice (fb 20260822T051709Z): the old
    margin came from det-band loadslip pricing (ok 2.2 taxed the
    churner's slip 2.39), and that same pricing is what zeroed the
    NOISY clone's income (ratio 5.91 at std 0.36) and shrank the gait
    — the two cannot coexist on one ratio threshold because the noisy
    clone measures HIGHER than the det attractor on the identical
    scalar. The residual det margin here is honest income difference
    (obey tracks the command better); det-band slip enforcement is
    owned by the eval gate (slip <= 1.15x clone) + the phase-locked BC
    anchor. KNOWN HOLE, recorded not hidden: at std 0.36 the NOISY
    fastcadence out-earns the noisy clone (~121 vs ~112) because mean-
    overdrive compensates noise and genuinely tracks the command
    better; no behavior-priced term can separate them (measured
    08-22). Containment = anchor + gate items (c)/(e)."""
    r = returns
    assert r[f"{bin_name}_obey"] > r[f"{bin_name}_fastcadence"] + 8.0, {
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
    """The obey-vs-attractor margin must come from the NEW alignment
    terms: under the phasedir1 stack (course/overspeed/idle removed)
    the margin was too small to stop PPO. SKEW ONLY since the
    phasedir3 reprice: the course charge still prices off-course
    travel hard (noise-robust: EMA direction + the 0.04 m/s smoothed-
    speed floor), but fastcadence's only det signature was band slip,
    whose pricing measurably taxed the noisy clone more than the
    attractor (see test_obey_beats_fastcadence_every_bin) — its
    attribution claim is retired WITH the pricing, honestly."""
    r = returns
    for drive in ("skew",):
        new_m = r[f"{bin_name}_obey"] - r[f"{bin_name}_{drive}"]
        old_m = r[f"old_{bin_name}_obey"] - r[f"old_{bin_name}_{drive}"]
        # +25 floor: measured 08-22 the course term takes fwd/skew
        # from ~90 to ~210+; the floor asserts real added pressure
        # without tuning reward doses to a test constant.
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
        # Pin the pre-reprice 0.01 floor: this unit test proves the
        # EMA itself spares stride sway (the launched stack's 0.04
        # floor would silence the slower compat-gait obey rollout
        # entirely and make the sway claim vacuous; the floor's
        # noise-tick behavior is pinned by the NOISY-REGIME tests).
        stack[("reward", "walk_course_min_speed_m_s")] = 0.01
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


# ---------------------------------------------------------------------------
# NOISY-REGIME tests (2026-08-22 phasedir3 reprice, operator focus
# fb 20260822T051709Z). phasedir2-staged-fwd's postmortem: PPO trains
# on STOCHASTIC rollouts (std stuck ~0.36), and the det-band charges
# taxed the exploration noise itself — training loadslip_ratio ~5.1
# zeroed the income gate all run, the |v_ema| overspeed band fired
# flat ~-3/tick, and the cheapest gradient was a shrunken gait
# (progress 0.836x clone, speed at the 0.060 floor). These tests pin
# the repriced stack IN THE TRAINING REGIME: the clone's own behavior
# under exploration noise must keep its income and pay ~no charge,
# and must out-earn the measured shrunken-gait basin, refusal, and
# parking. Launch bar per the focus note: no phasedir relaunch until
# these are green.


@pytest.fixture(scope="module")
def noisy_returns() -> dict[str, float]:
    out = {}
    for drive in ("obey", "shrunk", "stall", "park"):
        out[drive] = _mean(drive, HEADING_BINS["fwd"], PHASEDIR2_STACK,
                           noise_std=NOISE_STD)
    return out


def test_noisy_clone_keeps_income_and_pays_no_charges():
    """The clone's own gait under PPO exploration noise (std 0.36)
    must keep its loadslip income factor ~1 (phasedir2: 0.07) and pay
    near-zero loadslip-excess + overspeed charge over a 10 s episode
    (phasedir2: ~-286 loadslip alone, plus a flat |v_ema| overspeed
    tax). The course term must also stay small — its 0.04 m/s
    smoothed-speed floor exists exactly so noise-direction ticks are
    not priced (phasedir2 paid ~-263/episode of course tax at
    cos ~0.27)."""
    sums = []
    for s in SEEDS:
        c: dict = {}
        _phasedir_rollout("obey", s, 0.0, PHASEDIR2_STACK,
                          noise_std=NOISE_STD, collect=c)
        sums.append(c)
    for c in sums:
        charge = (c.get("reward_loadslip_excess", 0.0)
                  + c.get("reward_walk_course_overspeed", 0.0))
        assert charge > -20.0, (
            f"noise still taxed: loadslip+overspeed {charge:.1f}; {c}")
        assert c.get("reward_walk_course", 0.0) > -30.0, (
            f"course term still taxes noise: {c}")
        assert c.get("walk_loadslip_factor", 0.0) >= 0.9, (
            f"income gate still shut on the noisy clone: {c}")


def test_noisy_obey_beats_shrunken_gait():
    """THE preflight the phasedir3 launch is gated on (focus note:
    'launch fresh only after preflight proves progress-preserving
    behavior beats shrunken gait'): under exploration noise, driving
    the commanded 0.08 m/s must out-earn the measured failure basin
    (same command, gait shrunk to 0.75x). Under the phasedir2 stack
    this ordering was INVERTED (the shrink dodged noise-taxed
    charges); measured after reprice: obey ~112 vs shrunk ~95."""
    r_obey = _mean("obey", 0.0, PHASEDIR2_STACK, noise_std=NOISE_STD)
    r_shrunk = _mean("shrunk", 0.0, PHASEDIR2_STACK,
                     noise_std=NOISE_STD)
    assert r_obey > r_shrunk + 5.0, (r_obey, r_shrunk)


def test_noisy_obey_beats_refusal_and_park(noisy_returns):
    """Ordering must hold in the training regime too: the noisy clone
    out-earns marching in place and parking through the command.
    Margins are honest and thin (park ~102 vs obey ~112 — noise
    physically destroys most of the obedience income), so the bar is
    ordering + a small float-slack, not a fat margin constant."""
    r = noisy_returns
    assert r["obey"] > r["stall"] + 20.0, r
    assert r["obey"] > r["park"] + 3.0, r


def test_overspeed_along_projection_spares_sway_prices_directed():
    """Unit test for reward.walk_course_overspeed_along: on a REAR
    command at a LOW commanded speed (0.04 m/s), wrong-way +x travel
    at 0.08 m/s exceeds the |v_ema| band but its ALONG-command
    projection is NEGATIVE — the along variant must charge ZERO
    overspeed (the course term prices that travel, division of
    labor), while the legacy |v_ema| variant must charge it. Obedient
    travel at the same low command must clear the band either way."""
    from tripod_gait import TripodGait

    def overspeed_sum(drive: str, along: float) -> float:
        stack = dict(PHASEDIR2_STACK)
        stack[("reward", "walk_course_overspeed_along")] = along
        stack[("goal", "walk_speed_min_m_s")] = 0.04
        stack[("goal", "walk_speed_max_m_s")] = 0.04
        env = _make_walk_env(3, stack)
        env.reset()
        traj, n = _pin_command(env, math.pi, speed=0.04)
        gait = TripodGait(vx=0.0)
        gait.sync_plant_stance(*WALK_PLANT)
        gait.reset_phase()
        tot, step, t_gait = 0.0, 0, 0.0
        while True:
            i = min(step, n - 1)
            vx_ref, vy_ref = float(traj.vx[i]), float(traj.vy[i])
            if math.hypot(vx_ref, vy_ref) > 1e-3:
                t_gait += env.dt
                gv = ((vx_ref, vy_ref) if drive == "obey"
                      else (CMD_SPEED, 0.0))     # wrongway at 2x band
            else:
                gv = (0.0, 0.0)
            gait.set_velocity(vx=gv[0], vy=gv[1])
            act = q_rad_to_action(
                np.asarray(gait.desired_deg(t_gait)) * DEG2RAD)
            _o, _r, term, trunc, info = env.step(act)
            tot += float(info.get("reward_walk_course_overspeed", 0.0))
            step += 1
            if term or trunc:
                break
        env.close()
        return tot

    wrong_legacy = overspeed_sum("wrongway", 0.0)
    wrong_along = overspeed_sum("wrongway", 1.0)
    obey_along = overspeed_sum("obey", 1.0)
    assert wrong_legacy < -30.0, (
        f"legacy |v_ema| band never fired on 2x wrong-way travel "
        f"(bad test setup): {wrong_legacy}")
    assert wrong_along > -1e-6, (
        f"along projection charged wrong-way travel: {wrong_along}")
    assert obey_along > -5.0, (
        f"along projection taxes obedience: {obey_along}")


# ---------------------------------------------------------------------------
# PHASEDIR8 (2026-08-22 dig-in, phasedir7/7b step-function root cause).
#
# Measured on the ACTUAL checkpoints (logs/ckpt_eval/
# pd7b_digin_stancedist/*.json + the decomposition A/B recorded in the
# phasedir8 ledger entry), two coupled pricing defects:
#
#   A. drag_stance_allow_mm=6 sat BELOW the honest tibia-150 gait's
#      per-stance travel tail (clone pooled p90 12.3 mm, p95 19 mm):
#      the honest clone paid 2.87x its ENTIRE income in drag-stance
#      charge — travel itself was net-negative at ANY k, which is why
#      k=8000 and k=4000 produced the identical slow optimum (a step
#      function, not a dose curve). At allow=24 (clone ~p98) the
#      clone pays 0.10x income on ~2% of stances while the phasedir6
#      drag gait still pays 0.49x (the 08-11 audit intent restored).
#   B. the tracking kernel's INSTANTANEOUS 2D velocity error taxed
#      honest stride sway so hard that income was FLAT in realized
#      speed across [0.059, 0.08] m/s (slow pd7 gait kernel 357.1/ep
#      vs clone 325.6 — the sway tax cancelled k_prog's speed
#      payment). reward.walk_kernel_vel_ema=1 computes the kernel
#      error from a stride-EMA of body velocity (tau=0.75 s, the
#      course-term convention) so sway averages out; k_walk_prog=2
#      restores a clearly positive income slope in speed.
#
# Checkpoint ordering under THIS stack (det, seed 0, 15 s):
#   clone (0.0716 m/s) 1031  >  pd7 slow (0.0589) 978  >  pd6 drag 639
# The rows below pin what scripted drives CAN express (the learned
# drag regime itself cannot be scripted — pd5 dig-in lesson; the
# checkpoint A/B artifacts are the launch evidence for that part).
# ---------------------------------------------------------------------------

STD_TRAIN = 0.13                 # the warm-log-std-override regime
PHASEDIR8_STACK = dict(PHASEDIR2_STACK)
PHASEDIR8_STACK.update({
    # as launched since phasedir6 (band value refuted as a lever, kept)
    ("reward", "loadslip_ok"): 3.0,
    ("reward", "loadslip_max"): 6.0,
    # defect A repair: allowance recalibrated to the measured honest
    # tibia-150 per-stance tail (clone ~p98); k unchanged from pd7.
    ("reward", "k_drag_stance"): 8000.0,
    ("reward", "drag_stance_allow_mm"): 24.0,
    ("reward", "drag_stance_tick_floor_mm"): 0.25,
    # defect B repair: stride-EMA kernel + restored income slope.
    ("reward", "walk_kernel_vel_ema"): 1.0,
    ("reward", "walk_kernel_vel_tau_s"): 0.75,
    ("reward", "k_walk_prog"): 2.0,
})


def test_kernel_vel_ema_default_off_is_inert():
    """walk_kernel_vel_ema absent and =0.0 must be bit-exact equal
    (no reward change, no state leak into pricing)."""
    stack_off = dict(PHASEDIR2_STACK)
    stack_zero = dict(PHASEDIR2_STACK)
    stack_zero[("reward", "walk_kernel_vel_ema")] = 0.0
    r_off = _phasedir_rollout("obey", 0, 0.0, stack_off)
    r_zero = _phasedir_rollout("obey", 0, 0.0, stack_zero)
    assert r_off == r_zero, (r_off, r_zero)


@pytest.fixture(scope="module")
def pd8_returns() -> dict[str, float]:
    out = {}
    for drive in ("obey", "shrunk", "fastcadence", "stall", "park"):
        out[drive] = _mean(drive, HEADING_BINS["fwd"], PHASEDIR8_STACK)
    for drive in ("obey", "shrunk"):
        out[f"noisy_{drive}"] = _mean(
            drive, HEADING_BINS["fwd"], PHASEDIR8_STACK,
            noise_std=STD_TRAIN)
    return out


def test_pd8_obey_beats_slow_gait_det_and_noisy(pd8_returns):
    """THE step-function repair: commanded-speed walking must
    out-earn the 0.75x shrunken/slow basin (the class pd7/7b pinned
    into) both deterministically and at the training exploration std.
    Under the pd7 stack this ordering was inverted for the learned
    checkpoints (clone 684 < slow 806 at allow=20; worse at 6)."""
    r = pd8_returns
    assert r["obey"] > r["shrunk"] + 10.0, r
    assert r["noisy_obey"] > r["noisy_shrunk"] + 5.0, r


def test_pd8_anti_attractor_orderings_survive_reprice(pd8_returns):
    """Raising k_walk_prog and un-taxing sway must NOT resurrect the
    known attractors: obey still out-earns the cadence-churned speed
    chase, marching in place, and parking."""
    r = pd8_returns
    assert r["obey"] > r["fastcadence"] + 20.0, r
    assert r["obey"] > r["stall"] + 20.0, r
    assert r["obey"] > r["park"] + 20.0, r


def test_pd8_obey_drag_charge_is_small_fraction_of_income():
    """Defect A pinned: under the phasedir8 allowance the honest
    drive's drag-stance charge must be a SMALL fraction of its gross
    walk income (audit intent ~<=20-35%; at allow=6 the honest clone
    paid 287%). Uses the scripted obey drive — the honest gait class
    the allowance must spare."""
    for s in SEEDS:
        c: dict = {}
        _phasedir_rollout("obey", s, 0.0, PHASEDIR8_STACK, collect=c)
        income = c.get("reward_walk", 0.0) + c.get("reward_walk_prog", 0.0)
        charge = -c.get("reward_drag_stance", 0.0)
        assert income > 0.0, c
        assert charge < 0.35 * income, (
            f"allowance still taxes the honest gait: charge {charge:.1f}"
            f" vs income {income:.1f}; {c}")


def test_pd8_ema_kernel_restores_income_slope_in_speed():
    """Defect B pinned at the mechanism level: under the phasedir8
    stack the kernel+prog income of the commanded-speed drive must
    exceed the 0.75x slow drive's — i.e. income is no longer flat in
    realized speed. (Under the instantaneous kernel the slow gait's
    lower sway out-earned the clone on the kernel itself.)"""
    inc = {}
    for drive in ("obey", "shrunk"):
        tot = 0.0
        for s in SEEDS:
            c: dict = {}
            _phasedir_rollout(drive, s, 0.0, PHASEDIR8_STACK, collect=c)
            tot += (c.get("reward_walk", 0.0)
                    + c.get("reward_walk_prog", 0.0))
        inc[drive] = tot / len(SEEDS)
    assert inc["obey"] > inc["shrunk"] + 10.0, inc


# ---------------------------------------------------------------------------
# PHASEDIR9 (2026-08-22 phasedir8 dig-in, logs/ckpt_eval/pd8_digin_regime/).
#
# Measured on the ACTUAL checkpoints with probe_stance_slip_dist's new
# training-regime knobs (--action-noise-std / --dr-scale):
#
#   C. REGIME GAP: the det/DR-0 drag_stance calibration does not
#      transfer to the optimization regime. At allow=24/k=8000 the
#      honest phase clone pays 0.002-0.36x income DET but 0.76-9.7x
#      income at action noise std 0.135 alone (1.1-12x with
#      DR 0.35 + tipped starts) — travel was net-negative IN-REGIME
#      again, reproducing pd7's step-function optimum. Worse, NO
#      separating allowance exists: the noisy-honest per-stance tail
#      needs allow>=48mm untaxed while the pd6 det drag cheat pays
#      ZERO beyond 36mm (det 0.35-0.40x at 24). The repair is NOT a
#      band value — it is annealing the noise itself away
#      (train_ppo_mjx --log-std-final) so the optimization regime
#      converges to the det regime where the full-stack pricing is
#      measured-aligned (clone 1031 > pd7-slow 978 > pd6-drag 639).
#   D. RAMP MISFIRE (Warp-side): W&B on phasedir8 shows the EMA
#      overspeed band charging -2.38/charged-tick against a mean
#      exceedance of 0.002 m/s — arithmetically only possible on
#      ticks where s_ref is a few mm/s (the command ramp), where BOTH
#      the band threshold (1+tol)*s_ref and the fractional-exceedance
#      denominator collapse. reward.walk_course_overspeed_ref_floor_m_s
#      floors the reference: drift under (1+tol)*floor pays nothing,
#      full-command pricing (s_ref >= floor) is bit-identical.
# ---------------------------------------------------------------------------

PHASEDIR9_STACK = dict(PHASEDIR8_STACK)
PHASEDIR9_STACK.update({
    ("reward", "walk_course_overspeed_ref_floor_m_s"): 0.06,
})


def test_pd9_overspeed_ref_floor_default_off_is_inert():
    """ref_floor absent and =0.0 must be bit-exact equal, including
    under the training exploration noise (the charge path the floor
    touches only runs on noisy/low-command ticks)."""
    stack_off = dict(PHASEDIR8_STACK)
    stack_zero = dict(PHASEDIR8_STACK)
    stack_zero[("reward", "walk_course_overspeed_ref_floor_m_s")] = 0.0
    r_off = _phasedir_rollout("obey", 0, 0.0, stack_off,
                              noise_std=STD_TRAIN)
    r_zero = _phasedir_rollout("obey", 0, 0.0, stack_zero,
                               noise_std=STD_TRAIN)
    assert r_off == r_zero, (r_off, r_zero)


def _overspeed_sum_low_cmd(drive_speed: float, cmd_speed: float,
                           ref_floor: float, seed: int = 3) -> float:
    """Overspeed charge total for a straight-line +x teacher drive at
    drive_speed against a +x command pinned at cmd_speed — the
    low-command tick class is the scripted stand-in for ramp ticks
    (same s_ref scale, same charge formula)."""
    from tripod_gait import TripodGait

    stack = dict(PHASEDIR8_STACK)
    stack[("goal", "walk_speed_min_m_s")] = cmd_speed
    stack[("goal", "walk_speed_max_m_s")] = cmd_speed
    if ref_floor > 0.0:
        stack[("reward",
               "walk_course_overspeed_ref_floor_m_s")] = ref_floor
    env = _make_walk_env(seed, stack)
    env.reset()
    traj, n = _pin_command(env, 0.0, speed=cmd_speed)
    gait = TripodGait(vx=0.0)
    gait.sync_plant_stance(*WALK_PLANT)
    gait.reset_phase()
    tot, step, t_gait = 0.0, 0, 0.0
    while True:
        i = min(step, n - 1)
        s_ref = math.hypot(float(traj.vx[i]), float(traj.vy[i]))
        if s_ref > 1e-3:
            t_gait += env.dt
            gait.set_velocity(vx=drive_speed, vy=0.0)
        else:
            gait.set_velocity(vx=0.0, vy=0.0)
        act = q_rad_to_action(
            np.asarray(gait.desired_deg(t_gait)) * DEG2RAD)
        _o, _r, term, trunc, info = env.step(act)
        tot += float(info.get("reward_walk_course_overspeed", 0.0))
        step += 1
        if term or trunc:
            break
    env.close()
    return tot


def test_pd9_ref_floor_spares_ramp_class_prices_real_overspeed():
    """The misfire class: millimetre-scale drift against a tiny
    command (s_ref ~ ramp ticks) must stop paying under the floor,
    while genuinely fast directed travel at the same tiny command
    keeps paying real money (the insurance the band exists for).
    Teacher creep at 0.02 m/s vs a 0.01 m/s command emulates the
    drift-vs-ramp geometry (drift > (1+tol)*s_ref but far under the
    floored threshold). The mechanism is exercised at floor=0.03
    because no raw-teacher drive can sustain >0.063 along at the
    plant (bank honesty note: 2.5-3.5x drives saturate ~0.05) — the
    run-level floor 0.06 is cleared by the measured 0.139 m/s
    attractor by construction (0.139 > 1.05*0.06)."""
    drift_nofloor = _overspeed_sum_low_cmd(0.02, 0.01, 0.0)
    drift_floor = _overspeed_sum_low_cmd(0.02, 0.01, 0.03)
    fast_floor = _overspeed_sum_low_cmd(0.10, 0.01, 0.03)
    assert drift_nofloor < -2.0, (
        f"misfire class not reproduced (bad setup): {drift_nofloor}")
    assert drift_floor > -1e-6, (
        f"floor failed to spare drift-scale travel: {drift_floor}")
    assert fast_floor < -10.0, (
        f"floor killed the real-overspeed insurance: {fast_floor}")


def test_pd9_full_command_pricing_bit_identical_under_floor(pd8_returns):
    """At the full 0.08 m/s command s_ref >= floor, so every reward
    path must be bit-identical: the pd9 stack's det obey return equals
    the pd8 stack's (the floor only exists below 0.06 m/s)."""
    r9 = _mean("obey", HEADING_BINS["fwd"], PHASEDIR9_STACK)
    assert r9 == pd8_returns["obey"], (r9, pd8_returns["obey"])


def test_pd9_det_orderings_survive_ref_floor():
    """The pd8 anti-attractor orderings must survive the pd9 stack
    (the floor never touches full-command ticks, but assert the
    end-to-end orderings anyway — this is the stack a run launches
    with)."""
    r = {d: _mean(d, HEADING_BINS["fwd"], PHASEDIR9_STACK)
         for d in ("obey", "shrunk", "fastcadence", "stall", "park")}
    assert r["obey"] > r["shrunk"] + 10.0, r
    assert r["obey"] > r["fastcadence"] + 20.0, r
    assert r["obey"] > r["stall"] + 20.0, r
    assert r["obey"] > r["park"] + 20.0, r


def test_pd9_drag_regime_gap_pinned():
    """Finding C pinned at the mechanism level with the scripted
    teacher (the honest gait class): the SAME drive that pays a small
    drag fraction det pays a large one at the training noise std —
    the det calibration provably does not transfer to the noisy
    regime (the full checkpoint numbers live in
    logs/ckpt_eval/pd8_digin_regime/)."""
    det_c: dict = {}
    _phasedir_rollout("obey", 0, 0.0, PHASEDIR8_STACK, collect=det_c)
    noisy_c: dict = {}
    _phasedir_rollout("obey", 0, 0.0, PHASEDIR8_STACK,
                      noise_std=STD_TRAIN, collect=noisy_c)
    det_income = (det_c.get("reward_walk", 0.0)
                  + det_c.get("reward_walk_prog", 0.0))
    noisy_income = (noisy_c.get("reward_walk", 0.0)
                    + noisy_c.get("reward_walk_prog", 0.0))
    det_frac = -det_c.get("reward_drag_stance", 0.0) / max(det_income,
                                                           1e-9)
    noisy_frac = -noisy_c.get("reward_drag_stance", 0.0) / max(
        noisy_income, 1e-9)
    assert det_frac < 0.35, (det_frac, det_c)
    assert noisy_frac > 2.0 * det_frac + 0.1, (
        f"noise no longer fattens the drag bill? det {det_frac:.3f} "
        f"noisy {noisy_frac:.3f} — recheck before trusting the "
        f"log-std-anneal rationale")
