# amp - AMP locomotion from scratch

Last updated: 2026-08-22 (M1 discriminator core + bank landed).
Charter:
`rl_docs/AMP_LOCOMOTION.md` (binding, incl. the repo-adaptation
section — no Isaac Lab, MJX/Warp is the primary trainer). Keep this a
short screenful: Goal / Milestones / Now / Next.

## Goal

One compact learned policy, trained from scratch with Adversarial
Motion Priors + massively parallel PPO + privileged critic +
observation history + actuator/fault randomization, that:

- accepts continuous joystick commands (vx, vy, yaw_rate);
- produces a smooth alternating-tripod gait;
- starts, stops, reverses, strafes, turns without phase resets;
- recovers from pushes; degrades gracefully under joint/leg faults;
- transfers to plain MuJoCo unchanged (M5 = track DONE; M6 hardware
  is operator-owned).

The demonstration gait is training data, not the deployed controller.
Build every tool this needs; do not pause on operator input.

## Milestones (brief §13)

- M0 infrastructure: IN PROGRESS (actor/critic split + GRU/history +
  command generator + the widened joystick envelope all now confirmed
  composable on the primary trainer, 08-22 smoke; discriminator,
  motion library, replay buffer, and fault injection still open —
  see Now/Next)
- M1 motion library: IN PROGRESS (generator tool built + v1 dataset
  shipped 08-22; discriminator core + demo replay buffer + style
  reward now built and bank-tested 08-22, see Now — NOT YET WIRED
  into the live train_ppo_mjx reward loop, that integration is the
  next concrete step)
- M2 beautiful normal gait: NOT STARTED
- M3 push recovery: NOT STARTED
- M4 fault adaptation: NOT STARTED
- M5 MuJoCo transfer (= DONE gate): NOT STARTED

## Now

08-22 audit of the shared MJX/Warp stack against the M0 checklist
(§17 item 1: reuse before building) found more already reusable than
the 08-21 "nothing built yet" note assumed, plus one real gap that's
now closed:

- **DONE THIS CYCLE**: `--asym-critic` (privileged critic obs split)
  ported from `train_ppo_sim.py` (CPU/SB3) to `train_ppo_mjx.py`
  (the GPU/Warp primary trainer) — it did not exist there before
  (`git rev b69a46e2`). Reuses `asym_policy.AsymActorCriticPolicy`
  and `train_ppo_sim._privileged_idx` verbatim (no duplicated logic);
  additive-only (80 insertions, 0 deletions), new flag defaults off.
  Verified on-pod (`hexapod-mjx-train-0`, smoke `smoke-amp-
  asymcritic-mjx`): 2048/2048 steps trained, finite losses, and the
  saved checkpoint loads as `AsymActorCriticPolicy` with
  `privileged_idx=(70,71)` correctly masking the walk task's 2
  measured-velocity obs dims on the actor path only. Warm-start
  transplant path (MLP champion -> asym policy) ported too, mirroring
  train_ppo_sim's proven weight-copy.
- **ALREADY THERE (confirmed, not built this cycle)**: GRU/history
  actor + deterministic recurrent eval (`--gru`, `--gru-dual`,
  `--gru-experts`, `gru_policy.py`) and a causal-transformer
  alternative (`--transformer`) are already live on `train_ppo_mjx.py`
  — M0's "recurrent state resets correctly" checkbox is largely
  covered already. Domain rand (`domain_rand.py`), per-world model DR,
  canaries, eval/video logging, and desync are all live per
  `guardrails.yaml`.
- **ALREADY THERE, PARTIAL**: `walk_task.py`'s existing command
  sampler (`goal.walk_cmd_mode=stress_mix` with
  `random_hold/flip_180/sweep_circle/square/stop_go/jitter`, plus
  `goal.walk_yaw_cmd` for yaw-rate) already draws continuous
  (vx, vy[, wz]) commands with hold/ramp/resample/stop segments —
  most of AMP §6's shape. Gaps vs. the brief: current defaults are a
  narrower (speed, heading) envelope, not the full independent
  vx in [-0.35,0.60] / vy in [-0.30,0.30] / yaw_rate in [-1.0,1.0]
  band with the brief's exact resample (0.5-3.0 s) / zero (0.15) /
  abrupt-change (0.35) probabilities, and the measured-velocity obs
  goes to BOTH actor and critic today — the new `--asym-critic` flag
  fixes the second half; the envelope is a cfg-tuning task, not new
  code.
- **DONE THIS CYCLE (M1 start)**: `rl_move/sim/build_motion_library.py`
  — generates + validates the §4 motion-prior dataset from the
  hardware-proven scripted teacher (`linux_control/tripod_gait.py`
  TripodGait) driven through REAL MuJoCo physics (not a bare
  kinematic replay) at the measured tibia-150 plant. Covers all of
  §4.2's required command families (forward x3 speeds, backward x2,
  lateral left/right, turn CW/CCW, forward+turn both signs, diagonal
  both signs, accel-from-rest, decel-to-rest) by driving the teacher's
  own vx/vy/omega directly — no mirroring transform needed, the sim is
  left/right symmetric so negative vy/omega already produces the true
  trajectory. Records the §3.6 discriminator feature set per tick
  (joint pos relative to a per-clip neutral, joint vel, base angular
  velocity, projected gravity, foot positions relative to the body)
  as `obs_style` (60-dim), plus phase/command/raw-pose metadata per
  §4.5. Validates each clip against the SAME slip/m and fall
  vocabulary the eval harness already uses elsewhere (reject on fall,
  dragging, or joint discontinuity) — no new pass/fail definitions.
  Shipped `rl_move/sim/motion_library/teacher_v1.npz` +
  `_manifest.json`: 15/15 clips accepted, 88 s (spec target 20-60 s),
  slip/m 0.76-1.43 for translational clips (inside the teacher's own
  1.4-2.9 hardware band or better) and 2.72-3.07 for the two
  turn-in-place clips (a rotation-as-speed proxy metric, noted as a
  known rough edge in the script). CPU-only (no GPU pod used).
  ASSUMPTION recorded (`OPERATOR_QUESTIONS.md` q_20260822T0900Z):
  "neutral pose" is per-clip (that clip's own post-reset spawn stance),
  not one global constant — whoever builds the discriminator must
  confirm/override this to match the policy's own obs convention;
  the raw (non-relative) joint_position array is kept in the npz so
  this can be redone without re-running the sim.
- **DONE THIS CYCLE (08-22, discriminator core)**:
  `rl_move/sim/amp_discriminator.py` — `MotionLibrary` (loads
  `teacher_v1.npz`, samples real (s_t, s_t1) transitions that never
  cross a clip boundary, fits dataset mean/std for input
  normalization per §3.6), `AMPDiscriminator` (plain-torch MLP over
  the concatenated 2x60-dim normalized transition, matching
  `asym_policy.py`'s no-framework-beyond-torch style), least-squares
  GAN `style_reward` (bounded [0,1], no reward cliff for a
  not-yet-competent policy) and `discriminator_loss` (+ R1 gradient
  penalty on real transitions only — the exact mechanism the brief
  asks for "so the style reward does not saturate immediately").
  Bank-tested (`rl_move/tests/test_amp_discriminator.py`, 8/8 PASS,
  CPU, ~10s): after a short training loop the discriminator
  separates held-out real motion-library transitions from BOTH i.i.d.
  noise fakes and temporally-shuffled fakes (mean D(real) > D(fake) +
  0.5 margin in both cases), gradient penalty and loss stay finite
  throughout (no instant-saturation blowup), style reward correctly
  prefers real transitions. Standalone CLI smoke
  (`python3 -m rl_move.sim.amp_discriminator`) confirmed live: loss
  1.09->0.40 over 200 steps, D(real) 0.04->0.57, D(fake) 0.03->-0.63.
  **NOT YET WIRED into train_ppo_mjx's live reward loop** — that
  requires computing this SAME 60-dim `obs_style` feature vector from
  the batched Warp/MJX env each rollout tick. Scoped but not started:
  the live env (`walk_task.py`) exposes `self.data.xpos` etc. on the
  CPU/eval path only; the GPU/Warp vec-env path train_ppo_mjx actually
  trains on does not currently expose per-foot Cartesian positions or
  projected gravity as a reusable array the way `build_motion_library.py`
  computes them off the CPU sim. Candidate lower-risk route (not yet
  attempted): derive `obs_style` from the ACTOR'S OWN observation
  vector at each step (which already contains joint pos/vel, IMU
  angular velocity, projected gravity per §3.2) plus a forward-
  kinematics-only foot-position reconstruction (reusing
  `tripod_gait.foot_rz_from_hip_knee`/`_leg_ik`, as `extract_rise_ref.py`
  already does), avoiding any new Warp-kernel-level work — this needs
  its own dedicated cycle to build+bank-test the obs-vector-to-
  obs_style mapping (index-matching the exact actor obs layout) before
  it touches the reward loop, default-off, bit-exact-when-off per the
  standing rule.
- **CONFIRMED NOT STARTED**: demo replay-buffer <-> PPO co-training
  loop, fault injection, push-disturbance curriculum, and the
  dedicated joystick eval suite (`eval/joystick_script.py` etc. per
  brief §15). Motion library v1 has no augmentation yet (mirroring/
  speed/phase scaling per §4.2) — the 15-family/1-seed base already
  clears the 20-60s target so augmentation is not currently a
  blocker, only a future diversity improvement.

## Next (brief §17 order — M0/M1)

1. **DONE 08-22** (`amp-m0-joycmd-asymcritic-smoke-v3`, W&B-disabled
   infra smoke, n_envs=4096, 500k steps, train-0): confirmed
   `--asym-critic` composes end-to-end with a FRESH (from-scratch,
   no `--init-from`) policy over a widened `stress_mix` cfg bundle
   matching AMP §6's envelope — `goal.walk_speed_min/max_m_s=0.0/0.60`,
   full-circle heading (default `walk_heading_max_rad=-1`),
   `goal.walk_yaw_cmd=1` + `walk_yaw_max_rad_s=1.0` +
   `walk_yaw_zero_frac=0.15`, `walk_cmd_resample_s=1.75` +
   `walk_cmd_resample_jitter=0.714` (spans exactly 0.5-3.0s),
   `walk_cmd_blend_s_min/max=0.05/1.0` (mixed abrupt/ramped
   transitions). Result: finite losses/values the whole run
   (value_loss ~29-32, explained_variance 0.64-0.76, std stable
   ~0.368, no NaN/crash), video reel ok, checkpoint verified on-pod
   loads as `AsymActorCriticPolicy` with the expected 73-dim actor
   obs space. TWO FAILED ATTEMPTS FIRST (v1/v2, both FAILED/logged):
   warm-starting from the joystick track's phase-clone checkpoint hit
   obs-width mismatches (the source checkpoint has neither the new
   yaw_cmd dim nor a compatible phase_obs width), and
   `--obs-pad-transplant` (the tool that would normally patch an
   obs-width change onto a warm start) is explicitly incompatible
   with `--asym-critic` — the fix was going from-scratch, which is
   the track's own charter default anyway, not a new tool. LESSON:
   AMP-track smokes must never `--init-from` a joystick-track
   checkpoint; obs-width plumbing only needs to agree with itself.
   The independent-vx/vy-vs-polar-speed+heading gap noted below is
   NOT a blocker — full-circle heading + speed magnitude covers
   reverse/lateral/diagonal commands functionally.
2. **DONE 08-22 (base library)**: motion-library generation from the
   teacher (all §4.2 command families) + validation (reject dragging/
   fall/joint-discontinuity clips) — see Now. STILL OPEN: mirroring +
   speed/phase augmentation (only needed if the discriminator turns
   out to want more than 88s / more per-clip diversity than the
   single-seed deterministic base gives it — cheap to add later,
   not gating discriminator work from starting).
3. **DONE 08-22 (core)**: `rl_move/sim/amp_discriminator.py` —
   discriminator, real-transition replay sampler, style reward,
   gradient penalty + input normalization, all consuming
   `teacher_v1.npz`'s `obs_style` field directly per §3.6/§4.5. Bank
   `test_amp_discriminator.py` 8/8 PASS (real-vs-noise AND
   real-vs-shuffled separation, finite gradient penalty, bounded
   style reward). STILL OPEN, next concrete step: compute `obs_style`
   from the LIVE train_ppo_mjx rollout so the discriminator can see
   policy-generated transitions (not just the demo library) — see
   the candidate route in Now (actor-obs + FK foot reconstruction,
   avoiding new Warp-kernel work), then a co-training loop that
   updates the discriminator alongside PPO each rollout.
4. Smoke test: gradients flow through PPO and the discriminator
   trains without instant saturation on POLICY rollouts specifically
   (item 3's bank already proved this against a static real-vs-fake
   split; this item is the live-rollout version, blocked on item 3's
   remaining wiring).
5. Wave 1 across 8 pods: 3 seeds at task/style 0.5/0.5, no-AMP
   ablation, recurrent vs fixed-history, higher/lower AMP weight.
   Select on videos + tracking/stability metrics, never scalar return.

## Required status block (update after each wave)

Current milestone / Best checkpoint / Code revision / Samples / Wall
time / FPS / Normal-gait, joystick, visual, push, fault, transfer
verdicts / Top 3 failures / Exact next experiments — per brief §18.
