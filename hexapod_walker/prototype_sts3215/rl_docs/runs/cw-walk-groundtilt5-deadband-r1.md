# cw-walk-groundtilt5-deadband-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T03:25:05+00:00

**pod**: hexapod-mjx-train-7

**steps**: 20000000

**parent**: cw-walk-groundtilt5

**wandb_id**: zfbdjsac

**hypothesis**: Retry (r1) of cw-walk-groundtilt5-deadband, which died 0-step to the fleet launch-collision storm (infra, not science). Same hypothesis: floor-slope (5deg) x servo deadband (1.0-3.0x) compose, untried pairing.

**gate**: Own-cfg (tilt u(0,5deg) + dr.deadband_scale=1.0,3.0) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd>=1.2m; DR0 flat no-deadband retention det 6/6 gv, slip/m<=1.24, prog>=0.90; frames watched det

**failed_reason**: W&B global_step not advancing (0 -> 0)

