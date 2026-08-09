# cw-walk-strafe-dr10

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T15:39:58+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-strafe-dr05

**wandb_id**: g5tt9k6r

**hypothesis**: DR ladder rung for the lateral/omni line (wishlist 9+5): strafe-dr05 held ±90° transport at DR 0.5 with slip better than its DR0 parent; one variable — train at model-DR 1.0 off the strafe-dr05 checkpoint. Brings the lateral base to the champion lineage's robustness level for the joystick envelope (P2). If-true: own-eval DR1.0 keeps gv 12/12, 0 term, prog med >=0.8, slip/m med <=2.4 — the lateral paddle is DR-stable to full strength. If-false: lateral transport collapses at full DR (prog craters or falls) — DR 0.5 is the strafe ceiling; lateral needs gait change, not more exposure.

**gate**: own-cfg DR1.0 6+6: gv 12/12, 0 term, prog_ratio med >=0.8, slip/m med <=2.4; DR0 det retention prog med >=0.8 gv 6/6; frames watched det

