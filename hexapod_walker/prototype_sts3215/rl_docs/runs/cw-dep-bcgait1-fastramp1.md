# cw-dep-bcgait1-fastramp1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: done

**created**: 2026-08-20T19:20:49+00:00

**pod**: hexapod-mjx-train-1

**steps**: 1000000

**parent**: cw-dep-bcgait1-hard1

**wandb_id**: l195bxd4

**hardware_ready**: False

**hypothesis**: Remove the servo-profile cliff with a ramp, at the FULL dose: this canary starts the proven tall walker at the fitted servo profile it is stable in (350 counts/s / acc 20 / 1.5-deg slew) and linearly anneals to the full 1500/80/5-deg target over the first 500k steps (bus.profile_ramp_steps=500000, half the 1M canary budget) while the fast anti-skate curriculum (V5) and direct loaded-slip penalty train. Operator order q_20260820T0830Z answer (MCP operator lane 20260820T191113Z): execute option (b) ramp-in; full-dose sibling launched in parallel per the capacity clause. Prior evidence: the transplant zero-shot FAILS B0 at this dose hard (fastnoslip1: falls 6/8, slip 2.26/m, 2.09x overshoot) and the failure was dose-graded, so the full dose is the stronger test of whether annealing defeats the cliff. Companion cw-dep-bcgait1-fastthru1 (same dose, precert waived, no ramp) separates ramp-in from train-through; mid-dose siblings midramp1/midthru1 read the dose axis.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature fast gait here. Pre-registered (operator pick (b), q_20260820T0830Z): (1) pre-PPO B0 cert at the ramp START profile PASSES (zero falls + cmd_prog_frac>=0.50; --walkcurr-cert-at-init retained, fail-closed - a step-0 FAIL falsifies 'the transplant is stable at its fitted profile'); (2) ramp completes by 500k (profile_ramp/frac -> 1.0) without canary auto-stop, tilt-exit storm, or KL-rollback storm; (3) by 1M, judged strictly AT THE FULL 1500/80/5-deg TARGET DOSE (cert env mirrors training frac; eval envs sit at target by design): B0 cert zero falls, cmd_prog_frac>=0.50, slip_per_m trending <=1.6, cross_track<=0.20, height_factor>=0.80; B1 improving is a bonus, not required; (4) health: finite losses, reward_loadslip_excess trending toward zero. FAIL if falls reappear as the ramp anneals and do not recover by the next cert round, or steer6-style skating recurs (slip/m>2.5, direction/cross-track failure). Compare against fastthru1 (same dose, option (a)) and midramp1 (mid dose, same mechanism). DOWNLOAD_ANSWER and the hierarchy baseline are untouched unless this arm passes its gate.

**verdict**: CANARY FAIL - MECHANISM: ramp-in does not fix the full-dose (1500/80/5deg) destabilization. Step-0 B0 bridge cert already FAILs cross_track+roll at the ramp START (fitted) profile, falsifying the premise it is stable there; by 1M at full target dose, own-DR eval TERMs walk_low_height 6/6 det trials, slip/m median 7.0 (thresh <=1.6), and gate eval shows dir_err 43-52deg (thresh cross_track<=0.20) -- reproduces the pre-named steer6-style skating stop condition. Contact-sheet video shows the robot spinning in place rather than translating toward the command.

