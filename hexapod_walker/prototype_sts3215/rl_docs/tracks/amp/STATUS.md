# amp - AMP locomotion from scratch

Last updated: 2026-08-22 (first M0 audit + code cycle). Charter:
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
  most of the command generator now exist on the primary trainer;
  fault injection + wired-up joystick env + auto video/metrics still
  open — see Now/Next)
- M1 motion library: NOT STARTED
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
- **CONFIRMED NOT STARTED**: AMP discriminator, demonstration replay
  buffer, style reward, motion-library generation/validation tooling,
  fault injection, push-disturbance curriculum, and the dedicated
  joystick eval suite (`eval/joystick_script.py` etc. per brief §15).
  Zero lines exist for any of these.

## Next (brief §17 order — M0/M1)

1. Wire a `goal.walk_cmd_mode=joystick` (or widen `stress_mix`) preset
   that matches AMP §6's exact envelope/probabilities on top of the
   existing sampler, plus `--cfg-set` defaults for the §16 skeleton;
   confirm `--asym-critic` composes with it end-to-end (n_envs>=1024
   smoke).
2. Motion-library generation from the teacher (all command families:
   fwd/back/lateral/turn/diagonal/accel/decel; mirroring + speed/phase
   augmentation) + validation metrics; reject dragging/collision clips.
3. AMP discriminator, demonstration replay buffer, style reward with
   gradient penalty + input normalization.
4. Smoke test: gradients flow through PPO and the discriminator
   trains without instant saturation (M0/M1 checks).
5. Wave 1 across 8 pods: 3 seeds at task/style 0.5/0.5, no-AMP
   ablation, recurrent vs fixed-history, higher/lower AMP weight.
   Select on videos + tracking/stability metrics, never scalar return.

## Required status block (update after each wave)

Current milestone / Best checkpoint / Code revision / Samples / Wall
time / FPS / Normal-gait, joystick, visual, push, fault, transfer
verdicts / Top 3 failures / Exact next experiments — per brief §18.
