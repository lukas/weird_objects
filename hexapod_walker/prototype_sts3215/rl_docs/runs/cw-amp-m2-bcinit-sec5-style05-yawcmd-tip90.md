# cw-amp-m2-bcinit-sec5-style05-yawcmd-tip90

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL-INFORMATIVE

**created**: 2026-08-22T22:45:52+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05-yawcmd

**wandb_id**: nqs2fby4

**hypothesis**: Plain English: the maximum-exposure dose twin of tip50-r2 -- if turn-in-place learning needs command DENSITY, making 90% of episodes whole-episode turns should show a clear dose-response over the 0.5 arm; the accepted cost is translation forgetting, which is fine for a curriculum STAGE (a later stage re-mixes translation back in; measured baseline: yawcmd det dir_err 32.9 / prog 1.22). Same substrate and single lever as tip50-r2: continue from the yawcmd checkpoint (74-dim obs + full bank-verified yaw pricing + teacher_v2 real turn clips), only goal.walk_turn_in_place_frac differs (0.9 vs 0.5). Prediction-if-true: tip-left/right err drops below tip50-r2's (dose-response) with command-signed wz both directions. Prediction-if-false: still parks at 90% exposure -- together with a tip50-r2 park this REFUTES command exposure as the unlock on this substrate and the next lever is structural (mirror-symmetry regularizer / turn-specific gait phase), not more exposure or pricing. Strongest alternative: turns emerge but gait degrades to a shuffle (style/gait conflict under rotation) -- read the tip-episode videos, not just wz.

**gate**: Discovery (2M, DR-0). PASS = eval_yaw (manual, on the pod, --speed 0.08 --wz-max 0.3 + run cfg) tip-left AND tip-right err <= 0.20 with achieved wz sign matching wz_ref both directions, zero falls, gait_valid >=5/6 on the standard DR-0 gate (translation erosion TOLERATED on this arm -- stage read, compare dir_err vs yawcmd det 32.9/sto 49.0 and report it, but it does not fail the arm). FAIL-park = tip errs ~0.30 both arms -> command exposure refuted on the AMP substrate, next lever structural. Read jointly with tip50-r2 for the dose-response.

**verdict**: Fires FAIL-park at the 0.9 dose too, completing the joint read with tip50-r2: COMMAND EXPOSURE IS REFUTED as the turn-in-place unlock on the AMP substrate. Even with 90% of training episodes as whole-episode dedicated turns (50/50 sign draw, the maximum-exposure arm where translation erosion was explicitly tolerated), eval_yaw (pod, run cfg, --speed 0.08 --wz-max 0.3; artifact logs/ckpt_eval/cw_amp_m2_bcinit_sec5_style05_yawcmd_tip90_yawgate.json) shows tip-left/right err 0.2982/0.2999 == |wz_ref| exactly -- the identical park fingerprint as the parent yawcmd (0.2995/0.3008) and the tip50-r2 twin (0.2996/0.3000); zero falls, hold_wz 0.0020 clean. No dose-response whatsoever between 0.5 and 0.9. Same secondary shape as the twin: yaw WHILE TRANSLATING did improve (fwd-hold drift err 0.169->0.075, arc-right 0.273->0.180) -- the policy modulates wz inside the walking gait but produces ZERO rotation from the parked stance at vx=0, at any exposure. Reward rose all run (quarters 43/103/160/186) while turning never emerged -- with 90% of income sitting on tip episodes the policy STILL prefers to park and collect style/still income; this is a motor-pattern DISCOVERY block, the same failure shape as the crouch-statue basin that 0/4 sec5 reward arms could not escape and only BC-init fixed. Standard DR-0 panel note: at 0.9 frac ~90% of harness episodes are tip commands the policy parks through, so panel medians are contamination, not gait erosion (see tip50-r2 verdict for the mechanism; its genuine translation episodes were fully preserved). Next (pre-registered structural branch, chosen assumption recorded in STATUS): inject the missing motor pattern directly -- extend bc_init_gait.py to drive the scripted teacher's native omega channel (TripodGait.set_velocity already supports it; the paper-CPG work measured teacher turns at yaw_along 0.99/1.01) under yaw-command episodes and BC-clone walk+turn, then RL with the sec5+yaw pricing stack from that clone. Mirror-symmetry regularizer stays the fallback if the turn-clone fails. hardware-ready: no (2M discovery, DR-0).

