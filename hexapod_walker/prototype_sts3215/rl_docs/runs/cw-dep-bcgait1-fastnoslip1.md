# cw-dep-bcgait1-fastnoslip1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-20T08:18:39+00:00

**pod**: hexapod-mjx-train-0

**steps**: 1000000

**parent**: cw-dep-bcgait1-hard1

**hardware_ready**: False

**hypothesis**: Teach the proven tall walker to walk FASTER without skating: this canary tests whether the new fast anti-skate curriculum (V5) plus a direct loaded-slip penalty lets the full-speed servo profile (write 1500/acc 80, 5-deg clamp) learn usable 0.06-0.10 m/s speed instead of the skating/steering failures that killed both prior fast-gait arms. Operator order fb_20260820T075230_4a90c6 (desktop patch recreated on controller, snapshot e1a81703): full 1500/80 STS profile has enough headroom, but only learns usable speed if V5 pays commanded forward progress (strict slip<=1.6/m, cross-track<=0.20, height>=0.80 cert gates) and reward.k_loadslip_excess=6.0 directly prices loaded slip in training. Warm full-checkpoint init from ppo_goal_cw_dep_bcgait1_hard1 (V5 adjacent-continuation exception); pre-PPO B0 bridge cert at the source's own 0.05-0.06 m/s operating point guards the transplant. Companion arm cw-dep-bcgait1-midnoslip1 (750/40, seed 22) separates motor-profile instability from reward/curriculum failure.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY (pre-registered, operator spec A in fb_20260820T075230_4a90c6): (1) pre-PPO V5 B0 cert survives with cmd_prog_frac>=0.50 (--walkcurr-cert-at-init + precert-min-prog, trainer aborts otherwise); (2) by 1M: B0 retained and B1 improving/promoted, slip_per_m trending <=1.6, cross_track<=0.20, height_factor>=0.80, zero falls; (3) training health: finite losses, no KL-rollback storm, reward_loadslip_excess trending toward zero. FAIL if steer6-style skating recurs: slip/m >2.5, direction/cross-track failure, or tilt exits. Judge mechanism health only — mature fast gait is judged at a full-budget follow-up, not here. Compare against companion midnoslip1 to separate profile dose from reward/curriculum effects.

**verdict**: CANARY FAIL - MECHANISM: pre-PPO gate clause (1) failed at step 0, zero training. The bcgait1_hard1 transplant does not survive V5 B0 (10s bridge at its own 0.05-0.06 m/s band) under the full 1500/80 profile — det n=8: falls 6/8, slip 2.26/m, roll 10.2 deg, prog 2.09 overshoot; trainer fail-closed abort per --walkcurr-cert-at-init. No behavioral/reward-class closure implied: V5+k_loadslip_excess untested by this arm; the question moves to companion cw-dep-bcgait1-midnoslip1 (750/40).

**failed_reason**: run never appeared as 'running' in W&B within 240s

