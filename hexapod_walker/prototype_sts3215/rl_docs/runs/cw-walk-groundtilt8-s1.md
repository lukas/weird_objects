# cw-walk-groundtilt8-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T01:46:12+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-groundtilt8-r3

**wandb_id**: ni1z6n7u

**hypothesis**: Ruling-7 seed twin of cw-walk-groundtilt8-r3 (this cycle PASS-with-caveat: 8deg tilt holds median but 3/6 det draws crater to a shuffle, worse fraction than groundtilt5's 2/6). Confirms whether the crater pattern is a real physics/exposure-ceiling effect (should reproduce at similar severity) or seed luck.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.ground_tilt_deg=8.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.1m, 0 falls/terminations even on steepest draws; DR0 flat retention det 6/6 gv, slip/m <=1.24; compare crater fraction (currently 3/6) to seed0

**failed_reason**: W&B global_step not advancing (0 -> 0)

