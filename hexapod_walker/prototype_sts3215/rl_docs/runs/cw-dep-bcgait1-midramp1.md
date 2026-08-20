# cw-dep-bcgait1-midramp1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: done

**created**: 2026-08-20T19:19:12+00:00

**pod**: hexapod-mjx-train-0

**steps**: 1000000

**parent**: cw-dep-bcgait1-hard1

**wandb_id**: fau0vd6t

**hardware_ready**: False

**hypothesis**: Remove the servo-profile cliff with a ramp instead of a waiver: this canary starts the proven tall walker at the fitted servo profile it is stable in (350 counts/s / acc 20 / 1.5-deg slew) and linearly anneals the live write profile to the mid 750/40/3-deg target over the first 500k steps (bus.profile_ramp_steps=500000, half the 1M canary budget) while the fast anti-skate curriculum (V5) and the direct loaded-slip penalty train. Operator order q_20260820T0830Z answer (MCP operator lane 20260820T191113Z): execute option (b) profile ramp-in alongside option (a) train-through as matched arms. Prior evidence: the same transplant zero-shot FAILS the B0 bridge cert at the target dose (midnoslip1: falls 2/8, roll 10.6 deg) but is stable at the fitted profile it trained under, so the ramp should let it cert cleanly at start and adapt as the dose rises. Companion arm cw-dep-bcgait1-midthru1 (same dose, precert waived, no ramp) separates ramp-in from train-through causality; full-dose siblings fastramp1/fastthru1 read the dose axis.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature fast gait here. Pre-registered (operator pick (b), q_20260820T0830Z): (1) pre-PPO B0 cert at the ramp START profile PASSES (zero falls + cmd_prog_frac>=0.50; --walkcurr-cert-at-init retained, fail-closed - a step-0 FAIL here falsifies 'the transplant is stable at its fitted profile' and is a real result); (2) ramp completes by 500k (profile_ramp/frac -> 1.0) without canary auto-stop, tilt-exit storm, or KL-rollback storm; (3) by 1M, judged strictly AT THE FULL 750/40/3-deg TARGET DOSE (cert env mirrors the training frac; eval/checkpoint envs sit at target by design): B0 cert zero falls, cmd_prog_frac>=0.50, slip_per_m trending <=1.6, cross_track<=0.20, height_factor>=0.80; B1 improving is a bonus, not required; (4) health: finite losses, reward_loadslip_excess trending toward zero. FAIL if falls reappear as the ramp anneals and do not recover by the next cert round, or steer6-style skating recurs (slip/m>2.5, direction/cross-track failure). Compare against midthru1 (same dose, option (a)) for clean A/B causality. DOWNLOAD_ANSWER and the hierarchy baseline are untouched unless this arm passes its gate.

**verdict**: CANARY FAIL - MECHANISM: ramp-in does not fix the mid-dose (750/40/3deg) destabilization either. Step-0 B0 bridge cert already FAILs cross_track+roll at the ramp START (fitted) profile; by 1M at target dose, DR-0 gate eval scores 0/6 walk success both det/sto, dir_err 43.5-46.8deg (thresh cross_track<=0.20), slip/m 2.3-4.8 (thresh <=1.6), roll settled only 1/6 -- reproduces the pre-named steer6-style skating stop condition, worse than the parent. Contact-sheet video shows the robot spinning in place rather than translating toward the command; session harness also FAILs (no_falls, rise, sit_descends).

