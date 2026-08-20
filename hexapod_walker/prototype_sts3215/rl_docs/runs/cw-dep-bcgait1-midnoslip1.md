# cw-dep-bcgait1-midnoslip1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-20T08:37:15+00:00

**pod**: hexapod-mjx-train-1

**steps**: 1000000

**parent**: cw-dep-bcgait1-hard1

**hypothesis**: Teach the proven tall walker to walk FASTER without skating: this canary tests whether the new fast anti-skate curriculum (V5) plus a direct loaded-slip penalty lets a MILDER motor profile (write 750/acc 40, 3-deg clamp -- about half the speed jump of the full-dose sibling) learn usable 0.06-0.10 m/s speed instead of the skating/steering failures that killed both prior fast-gait arms. Operator order fb_20260820T075230_4a90c6 spec B (desktop patch recreated on controller, snapshot e1a81703; branch pushed and diffed at origin/codex/recover-retention 2cb2a7b7 -- B0 bucket definition confirmed identical): the milder 750/40 profile is closer to the champion's native 400/20 training profile than the full 1500/80 dose, so it should survive the pre-PPO B0 bridge cert that the full-dose sibling (cw-dep-bcgait1-fastnoslip1) just failed with 75% falls at step 0. Warm full-checkpoint init from ppo_goal_cw_dep_bcgait1_hard1 (V5 adjacent-continuation exception); pre-PPO B0 bridge cert at the source's own 0.05-0.06 m/s operating point guards the transplant. Companion/disambiguation arm for cw-dep-bcgait1-fastnoslip1 (1500/80, seed 21) -- separates motor-profile-magnitude instability (does ANY speed increase need to be trained through gradually) from reward/curriculum failure (is V5 itself broken).

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY (pre-registered, operator spec B in fb_20260820T075230_4a90c6): (1) pre-PPO V5 B0 cert survives with cmd_prog_frac>=0.50 AND zero falls (--walkcurr-cert-at-init + precert-min-prog, trainer aborts otherwise -- this is the exact check the full-dose sibling failed); (2) by 1M: B0 retained and B1 improving/promoted, slip_per_m trending <=1.6, cross_track<=0.20, height_factor>=0.80, zero falls; (3) training health: finite losses, no KL-rollback storm, reward_loadslip_excess trending toward zero. FAIL if steer6-style skating recurs: slip/m >2.5, direction/cross-track failure, or tilt exits. Judge mechanism health only -- mature fast gait is judged at a full-budget follow-up, not here. Compare against fastnoslip1 (already CANARY FAIL - MECHANISM at the B0 precert) to read whether profile magnitude alone explains the failure.

**refused_reason**: W&B already has a run named cw-dep-bcgait1-midnoslip1 (names are append-only; pick a new one)

