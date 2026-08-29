"""WALK_COURSE_INCOME bank -- validates the windowed command-following
INCOME + excess-sway mechanism (operator reward-design directive
fb_20260829T142239_63c818, 2026-08-29).

THE DIRECTIVE, in one sentence: the training reward must PAY stable,
supported, commanded NET motion over gait-scale windows and must NOT
punish honest short-term tripod sway -- no raw per-tick heading/
cross-track objective; charge only sway EXCESS beyond a clean-teacher-
calibrated envelope.

Mechanism under test (walk_task.py):
  reward.k_walk_course_income -- per commanded tick,
      k * support_gate * angle_factor * speed_factor, where the
      angle/speed factors are computed on the NET body displacement
      over a trailing walk_course_income_window_s (default 0.75 s =
      teacher gait period) vs the command INTEGRATED over the same
      window (item 7: the integrated command IS the time-averaged
      reference across command switches; any stop tick inside the
      window pays nothing). angle_factor is 1 inside
      walk_course_income_deadband_deg and Gaussian beyond; the speed
      factor rises to 1 at full command completion and FALLS beyond
      (1 + walk_course_income_over_tol) -- the optimum sits AT the
      command (08-21 ruling), never above. support_gate is the product
      of the run's own configured anchor/loadslip/height/gait income
      gates (item 5) -- a shuffle/skate/flag-leg gait earns course
      income at the same discount its kernel income already takes.
  reward.k_walk_excess_sway -- RMS perpendicular path deviation around
      the commanded-course line over walk_sway_window_s, MINUS the
      teacher allowance walk_sway_allow_mm; only the positive excess is
      charged (item 4), and only around a roughly-followed course
      (walk_sway_course_cap_deg): sustained WRONG-course travel is
      priced by the income/course terms, not double-charged as sway.

TEACHER CALIBRATION (probe_dir_floor --envelope-windows, mesh/100 Hz/
0.375 deg-per-tick slew/0.08 m/s command, 2026-08-29; deterministic at
DR-0, identical across seeds):
  windowed course err   med 1.2-2.2 deg, p95 <= 5.2 deg (0.5-2 s)
  perpendicular sway    RMS p95 <= 1.7 mm
  along completion      ~0.39 of command (the scripted teacher CANNOT
                        reach 0.08 m/s under the mesh/100 Hz slew
                        contract; slip/m 1.27, zero falls, 6/6 legs)
  tick-level dir err    mean 13.5 / med 5.4 / p90 35.9 deg -- the
                        honest-sway noise floor the directive forbids
                        charging (and why the eval headline moved to
                        windowed course metrics, fb_20260829T141858).
Hence the defaults: deadband 6 deg (~teacher p95 + margin), sway allow
5 mm (~3x teacher p95). A raw |disp - cmd| vector kernel was REFUTED
before launch: at the teacher's own 0.39 completion the vector error is
speed-deficit-dominated (36 mm @ 0.75 s vs a park's 60 mm), so a
teacher-calibrated sigma would pay a PARK ~0.33 of max income -- the
angle x speed decomposition below is measured, not aesthetic.

MEASURED ORDERING under the candidate stack (8 s DR-0 mesh rollouts,
2026-08-29, this file's own fixture reproduces it):
  obey 1869 > fastcadence 1520 > zigzag 1187 > stall 693
  > sideways 593 > backward 254 > park 90
vs the directive's target chain teacher > zigzag > sideways/backward >
park/shuffle/skate/overspeed:
  - every mover beats PARK (true frozen refusal) -- required, holds;
  - honest teacher beats every cheat class -- holds; its angle factor
    is 1.0 (never charged for its own sway) and its excess-sway charge
    is exactly 0 -- the directive's central invariant;
  - DEVIATION (documented in OPERATOR_QUESTIONS.md 2026-08-29): a
    march-in-place STALL prices ~100 above SIDEWAYS and ~440 above
    BACKWARD. Pushing sustained wrong-way travel above stall would
    require either zeroing the negative-progress pricing other banks
    depend on or idle doses within ~2x of taxing the honest teacher's
    own 0.032 m/s crawl. Not traded away silently.
  - TRUE overspeed (achieved > 1.05x command) is PHYSICALLY
    UNREACHABLE by the scripted instrument on mesh at the 0.08
    command: an 8x-driven gait saturates at the same ~0.56 completion
    as 4x (measured, this cycle). The income speed-factor falloff
    beyond the band is armed regardless; a 4x "overdrive" that
    completes MORE of the command with CLEAN slip (1.6/m) legitimately
    out-earns the 1x teacher -- the optimum is the COMMAND, the 1x
    teacher parameterization is simply slow under this contract.
"""
from __future__ import annotations

import math
import os
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

CMD = 0.08
EP_SECONDS = 8.0

# The candidate stack: the standwalk-lineage support gates + refusal
# pricing, with the new windowed income/sway terms ON and the
# course_disp CHARGE at a deliberately small 0.15 dose (2.0 buried
# sideways/backward 300-1800 BELOW park, inverting the directive's own
# wrong-way > park requirement; measured 2026-08-29). Its overspeed
# twin keeps the full bank-proven 4.0 dose (independent cfg key).
STACK = {
    ("safety", "max_delta_q_deg"): 0.375,
    ("goal", "walk_speed_min_m_s"): CMD,
    ("goal", "walk_speed_max_m_s"): CMD,
    ("goal", "walk_heading_max_rad"): 0.0,
    ("reward", "walk_kernel_prog_gate"): 1.0,
    ("reward", "walk_anchor_gate"): 1.0,
    ("reward", "anchor_tol_mm"): 10.0,
    ("reward", "walk_height_gate"): 1.0,
    ("reward", "walk_height_sigma_mm"): 30.0,
    ("reward", "walk_loadslip_gate"): 1.0,
    ("reward", "loadslip_ok"): 3.0,
    ("reward", "loadslip_max"): 6.0,
    ("reward", "k_loadslip_excess"): 10.0,
    ("reward", "k_walk_idle_charge"): 20.0,
    ("reward", "walk_idle_speed_m_s"): 0.02,
    ("reward", "k_park_duty"): 2.0,
    ("reward", "k_drag_loaded"): 10.0,
    ("reward", "k_walk_course_income"): 2.0,
    ("reward", "walk_course_income_window_s"): 0.75,
    ("reward", "walk_course_income_deadband_deg"): 6.0,
    ("reward", "walk_course_income_sigma_deg"): 20.0,
    ("reward", "k_walk_excess_sway"): 2.0,
    ("reward", "walk_sway_window_s"): 0.75,
    ("reward", "walk_sway_allow_mm"): 5.0,
    ("reward", "k_walk_course_disp"): 0.15,
    ("reward", "walk_course_disp_window_s"): 1.5,
    ("reward", "walk_course_disp_min_speed_m_s"): 0.02,
    ("reward", "k_walk_course_disp_overspeed"): 4.0,
    ("reward", "walk_course_disp_overspeed_tol"): 0.05,
    ("reward", "walk_course_disp_overspeed_along"): 1.0,
    ("reward", "walk_course_disp_overspeed_ref_floor_m_s"): 0.06,
}

OFF_STACK = {k: v for k, v in STACK.items()
             if k[1] not in ("k_walk_course_income", "k_walk_excess_sway",
                             "k_walk_course_disp",
                             "k_walk_course_disp_overspeed")}

DRIVES = ("obey", "zigzag", "sideways", "backward", "stall", "park",
          "fastcadence", "overdrive")

INFO_SUM_KEYS = ("reward_walk_course_income", "reward_walk_excess_sway")
INFO_MEAN_KEYS = ("walk_course_income_angle_f", "walk_course_income_speed_f",
                  "walk_course_income_support", "walk_sway_rms_mm")


@pytest.fixture(scope="module", autouse=True)
def _mesh_family():
    """This bank is calibrated on the CURRENT defaults (mesh family,
    100 Hz) per the directive's own item 8 -- override the suite-wide
    primitive pin for this module only, restore after."""
    old = os.environ.get("HEXAPOD_MODEL_SOURCE")
    os.environ["HEXAPOD_MODEL_SOURCE"] = "mesh"
    yield
    if old is None:
        os.environ.pop("HEXAPOD_MODEL_SOURCE", None)
    else:
        os.environ["HEXAPOD_MODEL_SOURCE"] = old


def _rollout(drive: str, stack: dict, seconds: float = EP_SECONDS,
             seed: int = 0) -> tuple[float, dict]:
    # sim_gait_compat: the hardware-proven teacher dialect used by
    # probe_dir_floor / build_motion_library on the mesh family.
    from rl_move.config import load_config
    from rl_move.robot_state import DEG2RAD
    from rl_move.sim.joint_task import q_rad_to_action
    from rl_move.sim.servo_model import SimServoParams
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv
    from rl_move.sim.probe_walk_income import WALK_PLANT
    from sim_gait_compat import TripodGait

    cfg = load_config()
    for (sec, leaf), val in stack.items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=False,
        dr_scale=0.0, episode_seconds=seconds, seed=seed, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad", "walk"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "walk" else 0.0)
    env.reset()
    gait = TripodGait(vx=0.0, lift=0.025,
                      period_scale=(0.6 if drive == "fastcadence"
                                    else 1.0))
    gait.sync_plant_stance(*WALK_PLANT)
    gait.reset_phase()
    tot, t_gait = 0.0, 0.0
    coll: dict = {}
    while True:
        g = env._current_goal()
        vxr, vyr = ((float(g.vx_ref), float(g.vy_ref))
                    if g is not None else (0.0, 0.0))
        s_ref = math.hypot(vxr, vyr)
        if s_ref > 1e-3:
            t_gait += env.dt
            ux, uy = vxr / s_ref, vyr / s_ref
            if drive == "obey":
                gv = (vxr, vyr)
            elif drive == "fastcadence":
                gv = (1.74 * vxr, 1.74 * vyr)   # phasedir1 attractor x
            elif drive == "overdrive":
                gv = (4.0 * vxr, 4.0 * vyr)
            elif drive == "zigzag":
                # the unified-lineage failure class: net course roughly
                # followed, +/-40 deg heading flips each 0.75 s
                a = math.radians(40.0) * (
                    1 if int(t_gait / 0.75) % 2 == 0 else -1)
                gv = (s_ref * (ux * math.cos(a) - uy * math.sin(a)),
                      s_ref * (ux * math.sin(a) + uy * math.cos(a)))
            elif drive == "sideways":
                gv = (-s_ref * uy, s_ref * ux)
            elif drive == "backward":
                gv = (-vxr, -vyr)
            elif drive in ("stall", "park"):
                gv = (0.0, 0.0)
            else:
                raise ValueError(drive)
        else:
            gv = (0.0, 0.0)
        gait.set_velocity(vx=gv[0], vy=gv[1], omega=0.0)
        if drive == "park":
            act = q_rad_to_action(env._plant_deg * DEG2RAD)
        else:
            act = q_rad_to_action(
                np.asarray(gait.desired_deg(t_gait)) * DEG2RAD)
        _o, r, term, trunc, info = env.step(act)
        tot += float(r)
        for k in INFO_SUM_KEYS:
            if k in info:
                coll[k] = coll.get(k, 0.0) + float(info[k])
        for k in INFO_MEAN_KEYS:
            if k in info:
                coll.setdefault(k, []).append(float(info[k]))
        if term or trunc:
            break
    env.close()
    return tot, coll


@pytest.fixture(scope="module")
def bank() -> dict:
    out = {}
    for drive in DRIVES:
        out[drive] = _rollout(drive, STACK)
    return out


def test_income_and_sway_default_off_bit_exact():
    """k=0 with every other new key present must be bit-exact to the
    stack without any of the new keys: no state, no info keys."""
    r_off, c_off = _rollout("obey", OFF_STACK, seconds=4.0)
    zero = dict(OFF_STACK)
    zero[("reward", "k_walk_course_income")] = 0.0
    zero[("reward", "k_walk_excess_sway")] = 0.0
    zero[("reward", "walk_course_income_window_s")] = 2.0
    zero[("reward", "walk_sway_allow_mm")] = 0.5
    r_zero, c_zero = _rollout("obey", zero, seconds=4.0)
    assert r_off == pytest.approx(r_zero, abs=1e-9)
    assert not c_off and not c_zero


def test_teacher_rides_inside_its_own_envelope(bank):
    """The directive's central invariant: the clean teacher's honest
    stride sway is NEVER charged (angle factor pinned at 1 by the
    deadband, excess-sway charge exactly 0) and it collects real
    course income."""
    _, c = bank["obey"]
    assert float(np.mean(c["walk_course_income_angle_f"])) >= 0.99, c
    assert c.get("reward_walk_excess_sway", 0.0) == 0.0, c
    assert c["reward_walk_course_income"] > 300.0, c
    assert float(np.mean(c["walk_sway_rms_mm"])) < 2.0, c


def test_zigzag_sees_sway_charge_and_income_discount(bank):
    """The unified-lineage zigzag class must be visibly priced BOTH
    ways: a real excess-sway charge and a discounted (but nonzero --
    its net course is roughly right) course income."""
    _, c = bank["zigzag"]
    assert c["reward_walk_excess_sway"] < -150.0, c
    _, c_obey = bank["obey"]
    assert 0.0 < c["reward_walk_course_income"] \
        < 0.75 * c_obey["reward_walk_course_income"], (
        c["reward_walk_course_income"],
        c_obey["reward_walk_course_income"])


def test_wrong_course_and_refusal_earn_no_income(bank):
    for drive in ("sideways", "backward", "stall", "park"):
        _, c = bank[drive]
        assert c.get("reward_walk_course_income", 0.0) < 10.0, (drive, c)


def test_wrong_course_not_double_charged_as_sway(bank):
    """walk_sway_course_cap_deg: sustained wrong-course travel is
    priced by income/course terms, not the sway charge (double-
    charging inverted wrong-way > park; see module docstring)."""
    for drive in ("sideways", "backward"):
        _, c = bank[drive]
        assert c.get("reward_walk_excess_sway", 0.0) == 0.0, (drive, c)


def test_ordering_teacher_beats_every_cheat(bank):
    r = {d: bank[d][0] for d in DRIVES}
    assert r["obey"] > r["zigzag"] + 300.0, r
    assert r["obey"] > r["fastcadence"] + 150.0, r
    assert r["obey"] > r["stall"] + 700.0, r
    assert r["obey"] > r["park"] + 1200.0, r


def test_ordering_zigzag_above_wrong_course(bank):
    r = {d: bank[d][0] for d in DRIVES}
    assert r["zigzag"] > r["sideways"] + 300.0, r
    assert r["zigzag"] > r["backward"] + 500.0, r
    assert r["zigzag"] > r["stall"] + 250.0, r


def test_ordering_every_mover_beats_park(bank):
    r = {d: bank[d][0] for d in DRIVES}
    for drive in ("obey", "zigzag", "sideways", "backward", "stall",
                  "fastcadence"):
        assert r[drive] > r["park"] + 100.0, (drive, r)


def test_overdrive_clean_completion_legitimately_wins(bank):
    """A 4x-driven gait that completes MORE of the command with CLEAN
    slip out-earns the slow 1x teacher -- the optimum is the COMMAND
    (see module docstring; true above-band overspeed is unreachable by
    this instrument and its income falloff is unit-armed in code)."""
    r_over, c_over = bank["overdrive"]
    r_obey, c_obey = bank["obey"]
    assert r_over > r_obey, (r_over, r_obey)
    assert float(np.mean(c_over["walk_course_income_angle_f"])) >= 0.99
    assert c_over.get("reward_walk_excess_sway", 0.0) == 0.0
