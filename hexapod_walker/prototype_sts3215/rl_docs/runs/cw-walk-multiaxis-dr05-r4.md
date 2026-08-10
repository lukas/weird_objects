# cw-walk-multiaxis-dr05-r4

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T01:48:21+00:00

**pod**: hexapod-mjx-train-11

**steps**: 14000000

**parent**: cw-walk-multiaxis-dr05-r3

**wandb_id**: v7x6t7uc

**hypothesis**: Retry of cw-walk-multiaxis-dr05-r3 (died at init, 0 steps, fleet launch-collision gotcha 13b -- 4th attempt total, r1/r2/r3 all infra failures, no science result yet). Same spec unchanged: +14M step continuation of multiaxis-dr05 testing whether its DR0 retention miss (slip 1.27 vs 1.24 cap) is under-training or a hard stacking ceiling.

**gate**: Own-cfg (4-axis stack + DR0.5) det+sto 6/6: gait_valid 6/6, 0 term, prog med>=0.75; DR0 nominal retention det 6/6 gv, slip/m<=1.24, prog>=0.9; frames watched det.

**failed_reason**: W&B global_step not advancing (0 -> 0)

