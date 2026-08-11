"""Goal-conditioned lean / rise / weight-shift task on the MuJoCo twin.

One policy, one command interface: each episode gets a goal —

- ``hold``:   zero references (plain balance; the old task is a subset)
- ``lean``:   a constant roll/pitch target, ramped in over ~0.75 s
- ``track``:  slowly moving roll/pitch references (sums of slow sines) —
              deliberately excites the IMU and forces the policy to drive
              the body smoothly through the real servo dynamics
              (150-200 ms latency, deadband, acceleration ramp)
- ``unload``: drive one leg's servo currents to zero by shifting the
              body weight off it (the atomic prerequisite of stepping)
- ``raise``:  from the plant stance, follow a height ramp 10-30 mm UP —
              the deliberately trivial canary goal: a healthy setup
              should learn it to ~100% almost immediately
- ``rise``:   the episode STARTS in a crouch (body 30-70 mm below
              nominal); the height reference ramps up to nominal and the
              policy must stand the body up, then hold. Nothing pays
              until it moves — the anti-"freeze and collect alive" task.

The goal is appended to the observation (see ``TaskGoal.as_obs``), and
the shared ``compute_reward`` tracks the reference instead of zero. The
alive bonus is gated on tracking error so ignoring the goal doesn't pay.
References stay inside the body-IK action envelope (max_roll/pitch_deg,
max_height_mm) so every goal is physically reachable with all six feet
planted.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from rl_move.config import cfg_get
from rl_move.env import GOAL_DIM, TaskGoal
from .sim_env import N_OBS, SimHexapodBalanceEnv

DEG2RAD = math.pi / 180.0

try:
    import gymnasium as _gym
except ImportError:
    _gym = None


@dataclass
class GoalTrajectory:
    """Per-step references for one episode.

    ``height`` is relative to the EPISODE-START body height (same
    convention as the tilt refs, which are relative to start attitude).
    ``start_at`` tells the env which pose to reset in: "plant" (standing
    stance), "zero" (legs straight out, belly on the ground — how the
    operator places the robot), or "crouch" (feet at the plant footprint
    but body lowered by ``crouch_dz``).
    """
    mode: str
    roll: np.ndarray                 # (n_steps,) rad
    pitch: np.ndarray                # (n_steps,) rad
    height: np.ndarray               # (n_steps,) m, offset from start
    unload_leg: int | None
    lift_legs: tuple | None = None   # quad mode: legs to lift + keep clear
    start_at: str = "plant"
    crouch_dz: float = 0.0           # m below plant, for start_at="crouch"
    # Bridge starts (start_at="zero" only): 0 = flat belly pose, 1 = the
    # full crouch. Intermediate values start the robot partially curled
    # on its belly — a reverse curriculum from the solved crouch states
    # back toward the unsolved flat-belly start.
    start_curl: float = 0.0
    # For start_at="zero": fraction of the crouch pose to blend into the
    # start joints (0 = belly-flat zero pose, 1 = full crouch). Bridge
    # starts for the rise reverse-curriculum.
    start_curl: float = 0.0

    def at(self, step: int) -> TaskGoal:
        i = min(max(step, 0), len(self.roll) - 1)
        return TaskGoal(roll_ref=float(self.roll[i]),
                        pitch_ref=float(self.pitch[i]),
                        height_ref=float(self.height[i]),
                        unload_leg=self.unload_leg,
                        lift_legs=self.lift_legs)


class GoalGenerator:
    """Samples one episode's goal trajectory."""

    def __init__(self, cfg: dict):
        g = cfg.get("goal", {}) if isinstance(cfg, dict) else {}
        self.p_hold = float(g.get("p_hold", 0.10))
        self.p_lean = float(g.get("p_lean", 0.20))
        self.p_track = float(g.get("p_track", 0.20))
        self.p_unload = float(g.get("p_unload", 0.20))
        self.p_raise = float(g.get("p_raise", 0.15))
        self.p_rise = float(g.get("p_rise", 0.25))
        # "lower" (default OFF): the reverse of rise — from the standing
        # plant, follow a slow height ramp DOWN to belly rest without
        # banging. Enabled per-run via --goal-mix lower=<p>.
        self.p_lower = float(g.get("p_lower", 0.0))
        # "quad" (default OFF — operator party-tricks line, feasibility
        # GO at c57): from the plant, LIFT both front legs (0 and 5, per
        # quadruped_feasibility.FRONT_LEGS) and hold a level four-leg
        # stance. Command reaches the policy through the goal one-hot
        # (TaskGoal.lift_legs — obs width unchanged); the clearance/
        # planted reward lives in walk_task._post_step (cfg-gated
        # k_quad_*). Enable per-run via --goal-mix quad=<p>.
        self.p_quad = float(g.get("p_quad", 0.0))
        self.quad_legs = tuple(g.get("quad_lift_legs", (0, 5)))
        # References must be reachable through the body IK action limits.
        max_roll = float(cfg_get(cfg, "actions", "max_roll_deg", default=3.0))
        max_pitch = float(cfg_get(cfg, "actions", "max_pitch_deg", default=3.0))
        ref_cap = float(g.get("max_ref_deg", 2.5))
        self.max_roll = min(ref_cap, max_roll) * DEG2RAD
        self.max_pitch = min(ref_cap, max_pitch) * DEG2RAD
        period = g.get("track_period_s", [2.5, 8.0])
        self.period_s = (float(period[0]), float(period[1]))
        self.ramp_s = float(g.get("ramp_s", 0.75))
        rise = g.get("rise_height_mm", [30.0, 70.0])
        max_h = float(cfg_get(cfg, "actions", "max_height_mm", default=5.0))
        self.rise_m = (min(float(rise[0]), max_h) * 0.001,
                       min(float(rise[1]), max_h) * 0.001)
        self.rise_hold_s = float(g.get("rise_hold_s", 3.0))
        # Jitter floor for the pre-ramp hold (default = rise_hold_s, i.e.
        # no jitter). Setting it lower samples hold ~ U(min, rise_hold_s)
        # per episode so the policy learns to rise whenever the ramp
        # starts, instead of memorizing one curl-window choreography —
        # a fixed hold made early height commands fail outright.
        self.rise_hold_min_s = float(g.get("rise_hold_min_s",
                                           self.rise_hold_s))
        # Canary hook (not a cfg key): fixed-seed canary probes set this
        # to "flat"/"bridge"/"crouch" to pin the rise start kind for an
        # isolated eval episode. None (default) = normal random draw.
        self.force_rise_start: str | None = None
        # Rise start-kind mix (08-11, crouch-start lever): fraction of
        # rise episodes starting belly-flat / partial-curl; the crouch
        # fraction is the remainder. Defaults reproduce the hardcoded
        # 35/40/25 reverse curriculum exactly (legacy RNG stream is
        # unchanged — the same single r draw decides the kind). The
        # holdbc1 lineage's one residual defect is crouch-start rise
        # tip-overs (2/4 det at hard1); biasing the mix toward crouch
        # is the config-only lever to train against it.
        self.rise_flat_frac = float(g.get("rise_flat_frac", 0.35))
        self.rise_partial_frac = float(g.get("rise_partial_frac", 0.40))
        self.rise_ramp_s = float(g.get("rise_ramp_s", 4.0))
        raise_mm = g.get("raise_height_mm", [10.0, 30.0])
        self.raise_m = (min(float(raise_mm[0]), max_h) * 0.001,
                        min(float(raise_mm[1]), max_h) * 0.001)
        # Lower targets stay slightly shy of the full plant->belly drop
        # (~60 mm) so the commanded end height is physically reachable
        # without resting ON the ref error.
        lower_mm = g.get("lower_height_mm", [25.0, 55.0])
        self.lower_m = (min(float(lower_mm[0]), max_h) * 0.001,
                        min(float(lower_mm[1]), max_h) * 0.001)
        self.lower_hold_s = float(g.get("lower_hold_s", 1.0))
        # Fraction of lower episodes that start AT the belly-rest target
        # pose with a flat height ref ("rest here quietly") instead of
        # descending from the plant — reset-side basin injection, cycle
        # 24. Default 0.0 = feature off (legacy lower behavior).
        self.lower_belly_start_frac = float(
            g.get("lower_belly_start_frac", 0.0))
        # Slow on purpose: "gently, without banging" is the task. The
        # tracking kernel penalizes running ahead of the ramp, so a
        # 5 s descent IS the gentleness constraint.
        self.lower_ramp_s = float(g.get("lower_ramp_s", 5.0))

    def _ramp(self, n_steps: int, dt: float) -> np.ndarray:
        """Ease references in from 0 so episodes never start with a step
        change the servos (200 ms latency) could not possibly follow."""
        n_ramp = max(1, int(round(self.ramp_s / dt)))
        r = np.ones(n_steps)
        r[:n_ramp] = np.linspace(0.0, 1.0, n_ramp, endpoint=False)
        return r

    def _track_channel(self, rng: np.random.Generator, n_steps: int,
                       dt: float, amp_max: float) -> np.ndarray:
        t = np.arange(n_steps) * dt
        out = np.zeros(n_steps)
        for _ in range(int(rng.integers(1, 3))):
            amp = rng.uniform(0.3, 1.0) * amp_max
            period = rng.uniform(*self.period_s)
            phase = rng.uniform(0.0, 2.0 * math.pi)
            out += amp * np.sin(2.0 * math.pi * t / period + phase)
        # Sum of sines can exceed the reachable envelope: clip, then the
        # ramp (applied by caller) removes the initial discontinuity.
        return np.clip(out, -amp_max, amp_max)

    def sample(self, rng: np.random.Generator, n_steps: int,
               dt: float) -> GoalTrajectory:
        # "quad" appended with p=0 by default: a zero-probability entry
        # does not change Generator.choice's cdf mapping, so legacy rng
        # streams are unchanged.
        probs = np.array([self.p_hold, self.p_lean, self.p_track,
                          self.p_unload, self.p_raise, self.p_rise,
                          self.p_lower, self.p_quad])
        mode = str(rng.choice(
            ["hold", "lean", "track", "unload", "raise", "rise", "lower",
             "quad"],
            p=probs / probs.sum()))
        roll = np.zeros(n_steps)
        pitch = np.zeros(n_steps)
        height = np.zeros(n_steps)
        unload_leg: int | None = None
        start_at = "plant"
        ramp = self._ramp(n_steps, dt)
        if mode == "lean":
            roll = rng.uniform(-self.max_roll, self.max_roll) * ramp
            pitch = rng.uniform(-self.max_pitch, self.max_pitch) * ramp
        elif mode == "track":
            roll = self._track_channel(rng, n_steps, dt, self.max_roll) * ramp
            pitch = self._track_channel(rng, n_steps, dt,
                                        self.max_pitch) * ramp
        elif mode == "unload":
            unload_leg = int(rng.integers(0, 6))
        elif mode == "raise":
            # Canary: from the plant stance, ramp the height ref up and
            # hold. Slower than the tilt ramp (the servos travel farther)
            # but comfortably within the profile speed.
            target = rng.uniform(*self.raise_m)
            ramp_n = max(1, int(round(2.0 / dt)))
            height = np.full(n_steps, target)
            height[:ramp_n] = np.linspace(0.0, target, ramp_n,
                                          endpoint=False)
        elif mode == "lower":
            # Gentle descent: from the standing plant, follow a SLOW
            # ramp down to belly rest and stay there quietly. The height
            # kernel pays staying ON the ramp (not beating it down), the
            # gyro/action penalties price out banging, and the tilt trip
            # still terminates a topple.
            # Belly-rest reference states (goal.lower_belly_start_frac,
            # default 0.0 = feature off): with probability f the
            # episode instead STARTS flat on the belly (the zero pose —
            # which IS the lower target posture, all feet planted) with
            # height ref 0 throughout: "rest here quietly". Rationale
            # (cycle 24, endpost-c1 plateau): the terminal end-posture
            # charge stalled on a constant-cost manifold (leg 0/leg 4
            # clearance redistributed, sum ~unchanged ~130-150 mm over
            # two 4M segments) because planted-belly states are never
            # VISITED — the policy only approaches from above and stops
            # at its hover equilibrium; std 0.19 exploration cannot
            # reach contact. Putting the planted basin into the start
            # distribution teaches the posture (and its ~zero charge)
            # directly. The extra rng draw is unconditional so the
            # stream is identical between frac=0 and frac>0 runs of
            # THIS code (it does shift episode draws vs pre-cycle-24
            # code — same class of shift as any new sampled feature).
            belly = rng.random() < float(getattr(
                self, "lower_belly_start_frac", 0.0))
            if belly:
                start_at = "zero"
                height = np.zeros(n_steps)
            else:
                target = -rng.uniform(*self.lower_m)
                hold_n = max(1, int(round(self.lower_hold_s / dt)))
                ramp_n = max(1, int(round(self.lower_ramp_s / dt)))
                height = np.full(n_steps, target)
                height[:hold_n] = 0.0
                end = min(hold_n + ramp_n, n_steps)
                height[hold_n:end] = np.linspace(0.0, target, end - hold_n)
        crouch_dz = 0.0
        start_curl = 0.0
        if mode == "rise":
            # Reverse curriculum over the start pose: 35% belly-flat ZERO
            # (the real operator placement — must curl before any height
            # is reachable), 40% PARTIAL curl (start joints blended
            # 10-90% toward the crouch — a near-continuum from almost
            # flat to almost crouched), 25% full CROUCH (feet under
            # body, pure height-following). Run 03 (bridge 25-85% at
            # 30%) solved bridge states mid-run then regressed by the
            # end and never reached flat — the bottom rung was too far
            # off the floor and the rungs too sparse to consolidate.
            r = rng.random()
            force = getattr(self, "force_rise_start", None)
            if force is not None:
                # rng.random() above is still drawn so the stream is
                # identical whether or not the hook is armed. Bridge
                # maps to the middle of the partial band so the hook
                # stays correct under a reconfigured start mix.
                r = {"flat": 0.0,
                     "bridge": self.rise_flat_frac
                     + 0.5 * self.rise_partial_frac,
                     "crouch": 1.0}[force]
            rise = rng.uniform(*self.rise_m)
            hold_hi = self.rise_hold_s
            hold_lo = min(self.rise_hold_min_s, hold_hi)
            if r < self.rise_flat_frac:
                start_at = "zero"
                hold_s = float(rng.uniform(hold_lo, hold_hi))
            elif r < self.rise_flat_frac + self.rise_partial_frac:
                start_at = "zero"
                start_curl = float(rng.uniform(0.10, 0.90))
                crouch_dz = rise
                hold_s = float(rng.uniform(hold_lo, hold_hi))
            else:
                start_at = "crouch"
                crouch_dz = rise
                hold_s = 1.0
            hold_n = max(1, int(round(hold_s / dt)))
            ramp_n = max(1, int(round(self.rise_ramp_s / dt)))
            height = np.full(n_steps, rise)
            height[:hold_n] = 0.0
            end = min(hold_n + ramp_n, n_steps)
            height[hold_n:end] = np.linspace(0.0, rise, end - hold_n)
        lift_legs = self.quad_legs if mode == "quad" else None
        return GoalTrajectory(mode=mode, roll=roll, pitch=pitch,
                              height=height, unload_leg=unload_leg,
                              lift_legs=lift_legs,
                              start_at=start_at, crouch_dz=crouch_dz,
                              start_curl=start_curl)


class SimHexapodGoalEnv(SimHexapodBalanceEnv):
    """Goal-conditioned twin: 56-dim obs (47 + 9 goal), same 6-dim action."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._goal_gen = GoalGenerator(self.cfg)
        if _gym is not None:
            self.observation_space = self._obs_space_box(N_OBS + GOAL_DIM)

    def _sample_goal(self) -> GoalTrajectory:
        # +1: _current_goal is also read at step index == episode_steps.
        return self._goal_gen.sample(self.rng, self.episode_steps + 1,
                                     self.dt)

    def set_goal_mix(self, mix: dict) -> None:
        """Set p_<mode> sampling probabilities on the goal generator.
        VecEnv ``env_method`` hook — needed because the sharded MJX vec
        env's env objects live in worker processes, where in-process
        attribute pokes can't reach (train_ppo_mjx --goal-mix)."""
        for mode, p in mix.items():
            setattr(self._goal_gen, f"p_{mode}", float(p))


def make_goal_env(**kwargs):
    """Factory for SB3 ``make_vec_env``."""
    def _thunk():
        return SimHexapodGoalEnv(**kwargs)
    return _thunk
