# amp - AMP locomotion from scratch

Last updated: 2026-08-21 (track created by the operator two-track
reset). Charter: `rl_docs/AMP_LOCOMOTION.md` (binding, incl. the
repo-adaptation section — no Isaac Lab, MJX/Warp is the primary
trainer). Keep this a short screenful: Goal / Milestones / Now / Next.

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

- M0 infrastructure: NOT STARTED
- M1 motion library: NOT STARTED
- M2 beautiful normal gait: NOT STARTED
- M3 push recovery: NOT STARTED
- M4 fault adaptation: NOT STARTED
- M5 MuJoCo transfer (= DONE gate): NOT STARTED

## Now

Track just created. Nothing built yet. Reusable assets: the MJX/Warp
PPO stack (per-world model DR, canaries, eval/video, desync), the
asymmetric-critic flag, the scripted tripod teacher (clean at the
measured plant, slip/m 1.4-2.9) as the seed of the motion-prior
library, and the CPU MuJoCo eval harness as the independent
cross-engine validator.

## Next (brief §17 order — M0/M1)

1. Unified joystick-command env on the MJX stack with the
   actor/critic observation split (deployable actor obs only).
2. GRU/history actor support + deterministic recurrent evaluation.
3. Motion-library generation from the teacher (all command families:
   fwd/back/lateral/turn/diagonal/accel/decel; mirroring + speed/phase
   augmentation) + validation metrics; reject dragging/collision clips.
4. AMP discriminator, demonstration replay buffer, style reward with
   gradient penalty + input normalization.
5. Smoke test: gradients flow through PPO and the discriminator
   trains without instant saturation (M0/M1 checks).
6. Wave 1 across 8 pods: 3 seeds at task/style 0.5/0.5, no-AMP
   ablation, recurrent vs fixed-history, higher/lower AMP weight.
   Select on videos + tracking/stability metrics, never scalar return.

## Required status block (update after each wave)

Current milestone / Best checkpoint / Code revision / Samples / Wall
time / FPS / Normal-gait, joystick, visual, push, fault, transfer
verdicts / Top 3 failures / Exact next experiments — per brief §18.
