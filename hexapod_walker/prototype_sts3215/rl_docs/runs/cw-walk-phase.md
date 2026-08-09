# cw-walk-phase

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-08T20:37:54+00:00

**pod**: hexapod-sweep-long5m

**steps**: 4000000

**parent**: ppo_goal_cw_walk_dr04b.zip (md5 52220b24, in place)

**wandb_id**: o6a2x0u2

**hypothesis**: Phase-based alternating-tripod reward (Siekmann periodic reward composition, plan Walk item c): paying for clock-synchronized contacts every tick breaks the drag-shuffle that survived three fines (flag/flagw/speedhi refuted). If-true: phase_agreement >0.6, swings >=3/ep on ALL six legs, sacrificed_legs empty. If-false: agreement pins ~0.5 with shuffle intact, or agreement high while vel_err >0.05 (steps in place). Strongest alternative: clock-locked jerky stepping (fluidity judged in video). Arm (a) of the two-arm basin comparison (operator 20:25Z); arm (b)=cw-walk-phase-stance.

**gate**: sto walk >=4/6 gait-valid @ vel_err <=0.035 on 0.02-0.06 @ DR 0.4 AND video shows all six feet cycling contact/swing AND sto rise >=4/6 retained

**verdict**: FAIL (cycle 11c): walk det 0/6 gait-valid @ 0.033 / sto 0/6 @ 0.029, leg 3 parked 12/12, duty signature identical to parent; det rise regressed to 2/6 (flat 0/3); video shows the same flag-leg scoot. Phase reward did NOT restructure the converged shuffle at inherited low noise (std ~0.28) - matches the operator basin diagnosis; the same mechanism at std 1.0 from stance init drove six-leg cycling in the 96k probe. Warm-arm variant REFUTED; basin comparison completes with cw-walk-phase-stance2. NOT HARDWARE-READY. ckpt md5 80c091115f38d8b822251a3bb5273d86.

