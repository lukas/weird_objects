# MULTITASK — command-conditioned generalist locomotion (track: multitask)

Source: operator brief 2026-08-12 ("Hexapod RL: Multitask Learning,
Forgetting, and the Next Experiment"). This doc is the track's design;
live state is `rl_docs/tracks/multitask/STATUS.md`.

**NAMING CORRECTION (operator, 08-15, fb_20260815T114937_f9078d —
binding on the fb_20260815T114414_3c40d6 arm and all future runs):**
the c2-continuation arm is **`cw-joystick-translate1`** — the
user-facing task is joystick-commanded translation (policy receives
changing [vx,vy] and moves that way; wz/yaw identically zero and out
of the task description). "Fullcircle" is banned as a run/product
label (uniform [-pi,pi] heading is sampler coverage, not the
behavior); the launcher refuses it mechanically. Headline metric:
`joystick/v_along_m_s` (+`_cumulative`) = average signed m/s in the
requested joystick direction over nonzero-command ticks. Rule: name
the operator-visible behavior first; mechanism/sampler/arch details
live in config/tags/notes.

**METRIC SIMPLIFICATION (operator, 08-15, fb_20260815T115650_47010c —
applied 12:0x UTC, before launch):** the TRAINING dashboard carries NO
per-heading direction bins (`v_along_hbin*` removed from env info keys
and trainer W&B series; launcher refuses joystick launches if they
reappear). Uniform [-pi,pi] heading sampling + the RAW SIGNED average
already zeroes out command-ignorant motion, so bins add nothing in
training. Contract: `joystick/v_along_m_s` (per-rollout mean over
active ticks), `joystick/v_along_m_s_cumulative` (active-tick-weighted
run mean), `joystick/active_ticks` (audit count), episode
survival/fall metrics beside them; cross-track/wrong-way/ratio stay
secondary under `train/`. Fixed 8/12-direction panels are HELD-OUT
EVAL tools only, for diagnosing failures after the simple average
says whether learning is occurring.

## The reframe (read this before judging any run here)

Do NOT assume the campaign's main failure is catastrophic forgetting.
The stronger observed pattern: specialists PRESERVE their existing
behavior while FAILING TO ACQUIRE the new one (turn track: yaw never
arrived while forward stayed; tall ladder: height never arrived while
the paddle stayed). That is the signature of a narrow gait local
optimum, not of learn-new/forget-old. The forward champion is likely a
paddle/creep point solution without a smooth control direction for
yaw, lateral velocity, or speed — steering it may require dismantling
the very solution that earns its forward reward.

Corollary: `stand → champion → fine-tune forward → champion →
fine-tune turn → …` (continual fine-tuning) is the WRONG experiment
for "does task diversity make a smarter robot". PPO has no obligation
to preserve behavior absent from the current rollout distribution, and
it strongly inherits the previous specialist's optimum. This track
runs the right experiment instead.

Failure modes, ranked (from the brief): (1) narrow gait local optimum
— very likely; (2) task/reward-distribution interference between
branches — likely; (3) catastrophic forgetting — real but secondary;
(4) 128×128 capacity — plausible, unproven; (5) "PPO can't do
multitask locomotion" — unlikely (velocity/yaw-conditioned policies
are standard in legged RL).

## The experiment: fresh generalist branch

Start FROM SCRATCH (fresh init IS the hypothesis — the required
warm-start default is deliberately waived here). One policy,
command = [vx, vy, wz] in the observation, commands changing
mid-episode. Standing is the SAME task at command (0,0,0), not a
separate reward mode. One coherent reward: velocity tracking, yaw
tracking, uprightness, energy/action cost, calibrated anti-skate —
i.e. the existing shared kernel + walk income stack, unchanged. Never
toggle major reward terms because a semantic task label changed.

All of this maps onto EXISTING plumbing (walk_task.py; no new code for
wave 1): `goal.walk_speed_min/max_m_s`, `goal.walk_heading_max_rad`
(vy via heading), `goal.walk_yaw_cmd` + `walk_yaw_max_rad_s` +
`walk_yaw_zero_frac`, `goal.walk_cmd_resample_s` + `walk_stop_frac`
(stand = commanded stop segments), `goal.walk_obs_body_vel=2`.

## Wave 1 — the decisive A/B/C cohort (queued 08-12)

Matched everything (recipe cloned from `cw-dep-fresh1`, the proven
from-scratch dep-contract PASS: DR 0.2, 15 s episodes, ent 0.005,
log-std 0, anchor gate, 25° envelope, seed 0, 2M discovery); the ONLY
variable across arms is the command distribution. All three arms have
`walk_yaw_cmd=1` + resampling ON so obs layout and rng machinery are
identical (A just always draws wz=0 and never stops).

| arm | run | commands |
|-----|-----|----------|
| A specialist | `cw-mt-a1` | vx fixed 0.05, no vy, no yaw, no stops |
| B narrow generalist | `cw-mt-b1` | vx ∈ [0, 0.06]; wz ∈ ±0.15 rad/s (20% of segments); 40% stop (stand) segments |
| C broader generalist | `cw-mt-c1` | B + heading ≤ 0.34 rad (vy up to ≈ ±0.02 m/s) |

Command changes: resample every ~5 s (jitter 0.3), 1 s blends — the
policy must know what is requested; identical states require
different actions for stand vs walk vs turn.

## Evaluation (per checkpoint, every arm)

1. Fixed retained-command suite: stand, forward, small left yaw,
   small right yaw, mixed. Per-command tracking error, fall rate,
   progress, slip/contact, current, det+sto — recorded SEPARATELY,
   never one aggregate return.
2. Zero-shot interpolation probes (commands never sampled exactly):
   vx=0.037 wz=0.07 · vx=0.025 vy=0.012 · vx=0.05 wz=−0.11.
3. Video, always. A numerically fine but visibly paddle/creep gait is
   NOT evidence of a reusable locomotion representation. The single
   most informative outcome: does the fresh stand+forward+yaw policy
   develop a visibly different, more symmetric/phase-structured gait
   than the forward champion? If yes → the sequential objective built
   an unsteerable optimum. If it still paddles and cannot steer →
   investigate contact model, reward geometry, action representation,
   gait-phase structure BEFORE adding more tasks.

## Phase 2 — the transfer test (the original hypothesis, direct)

Checkpoint A/B/C. Give each the SAME genuinely new downstream command
(larger yaw, or backward motion). Fine-tune with fixed budgets
(1M / 2M / 5M), measuring (a) new-command performance and (b) retained
old-command suite at every checkpoint. Success criterion: "diverse
training makes the robot better at learning new tasks" is supported
ONLY if the generalist reaches the new-task target materially faster
than the specialist WHILE retaining useful old-command performance.
Never infer it from aggregate training return.

## Verdict labels (binding for this track)

- New command improves, retained commands regress materially →
  **interference/forgetting**. Not "generalization".
- Retained commands stay good, new command never improves →
  **acquisition/local-optimum failure**. Not "catastrophic forgetting".
- Failed branch → state the failure hypothesis FIRST, then pick an
  experiment that discriminates between hypotheses. No reflexive
  reward tweaks, no coefficient variants.
- The forward champion (`ppo_goal_cw_walk_longdist_r2`) is the frozen
  specialist baseline. Never overwrite or continue it in this track.

## Later waves (do not fold into wave 1)

- Capacity: keep 128×128 as the baseline; one generalist arm at
  256×256 (`--net-arch 256,256`) after wave 1 reads out. No
  transformers. `obs.history_frames` is a relevant axis (gait phase /
  actuator dynamics are partially observable) — separate arm.
- Body-height command; wider yaw; seed twins of the winning arm.
- Skill boundary: stand/stop, speed, steering, lateral, (later)
  height are ONE policy. Rise from unusual poses, self-righting,
  crash recovery stay SEPARATE skills for now — shared encoders /
  distillation only after each specialist is reliable.
- `k_drag_stance` (GAIT track's calibrated anti-paddle charge) is a
  wave-2 lever here, kept out of wave 1 so cohort failures map onto
  the known fresh1 lineage behavior.
