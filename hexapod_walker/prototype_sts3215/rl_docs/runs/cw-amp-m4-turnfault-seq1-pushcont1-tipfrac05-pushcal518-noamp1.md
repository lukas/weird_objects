# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-noamp1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T12:06:37+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

**wandb_id**: oum37524

**hypothesis**: Plain English: the pricing-nudge dose grid just closed 4/4 FAIL on both the yaw-tip and walk-slip axes (income-side dosing doesn't move either metric, and slip charge even moved the WRONG way) -- this diagnostic asks whether the AMP style term ITSELF is the structural cause, since the teacher_v2 motion library's turn_ccw/turn_cw/forward_turn_* clips are capped at only 0.20-0.25 rad/s while training/eval command turning up to 0.30 rad/s (a real, just-noticed mismatch: eval_yaw's tip-left/right commands sit right at that ceiling, and the achieved rate is only ~25-30% of commanded, more undershoot than the 0.25-vs-0.30 gap alone would predict but consistent with a discriminator that resists any motion pattern outside its narrow demonstrated range). Single lever vs pushcal518 (same seed/budget/push-recal/turn-fault composition): --amp-style-weight 0.5->0.0, which (per train_ppo_mjx.py L2043/2273) fully disables the AMPStyleVecWrapper construction -- a clean no-AMP ablation, same technique the wave-1 M2 plan already used as a standard control axis for this codebase.

**gate**: eval_amp_m5 walk+yaw sections (push/fault skip -- this checkpoint keeps push/fault DR on for training but the diagnostic only needs tracking). CONFIRMS AMP-style-as-cause if tip errs move toward the 0.20 bar by >=0.03 on both sides AND/OR walk det slip med improves toward 3.5 by >=0.15, with 0/12 raw falls preserved (safety must hold with AMP off too, since push-recovery style pressure might have been load-bearing for the push fix -- if falls reappear the AMP style term is helping stability, not just constraining turn/slip, and this becomes a wider re-open). REFUTES AMP-as-cause if tips/slip are unmoved (+-0.02 / +-0.15) or worse -- points the next dig-in toward gait-phase/stance-authority mechanisms instead of the discriminator.

**verdict**: The AMP-ablation diagnostic came back split, and it redirects the whole fork: turning got meaningfully BETTER with the style term off (so the discriminator is a real part of the turn cap), but slip got WORSE (so the style term is load-bearing for foot cleanliness and must stay). Evidence (eval_amp_m5 walk+yaw own-cfg, this dig-in cycle): tip_left_err 0.1778 (parent pushcal518 0.2157, -0.038, CLEARS the 0.20 bar), tip_right_err 0.2151 (parent 0.2351, -0.020, just misses), walk det slip med 3.919 vs parent 3.67 (+0.25 REGRESSION past the +-0.15 band, sto slip 4.85), 0 terms / gait_valid 12/12 / 0 yaw falls. Training telemetry matches: walk_yaw_err last10 0.257 vs priced arms' 0.256 (raw exposure unchanged; the eval-side tip gain is the discriminator pressure coming OFF at the achieved-rate margin). ROOT CAUSE, measured this cycle by direct probe of the scripted teacher: the untrimmed tripod gait ACHIEVES only 0.134-0.144 rad/s at commanded 0.25-0.30 (ratio 0.48-0.54 - exactly the price-invariant ~0.5 ratio every tipfrac05 policy is pinned at), saturating ~0.15-0.16 at any commanded omega because foot sweep per stride is geometry-capped; teacher_v2's turn demos therefore EMBODY ~0.134 rad/s while labeled 0.25, and the discriminator (obs_style includes base_angular_velocity, unconditioned on command) punishes any rotation faster than the demos. Next: keep AMP on and fix the demos - teacher_v3.npz built+snapshotted this cycle (turn clips at stride_scale 1.4/period_scale 1.2 = measured 0.174 rad/s, +30% demo ceiling, slip/m 1.26, non-turn clips bit-exact); arm -turnlib3 launches on it. Slip axis gets the full-strength recalibrated loadslip income gate (-loadgate45) since AMP-off made slip worse and additive pricing is refuted.

