# cw-walk-strafe-dr10

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T15:39:58+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-strafe-dr05

**wandb_id**: g5tt9k6r

**hardware_ready**: no

**hypothesis**: DR ladder rung for the lateral/omni line (wishlist 9+5): strafe-dr05 held ±90° transport at DR 0.5 with slip better than its DR0 parent; one variable — train at model-DR 1.0 off the strafe-dr05 checkpoint. Brings the lateral base to the champion lineage's robustness level for the joystick envelope (P2). If-true: own-eval DR1.0 keeps gv 12/12, 0 term, prog med >=0.8, slip/m med <=2.4 — the lateral paddle is DR-stable to full strength. If-false: lateral transport collapses at full DR (prog craters or falls) — DR 0.5 is the strafe ceiling; lateral needs gait change, not more exposure.

**gate**: own-cfg DR1.0 6+6: gv 12/12, 0 term, prog_ratio med >=0.8, slip/m med <=2.4; DR0 det retention prog med >=0.8 gv 6/6; frames watched det

**verdict**: FAIL — DR1.0 gate missed on the if-false side: own-cfg DR1.0 gv 11/12 (det ep2 sacrifices legs 4+5: prog 0.36, slip/m 5.79 — a real flag-leg pathology, frames watched), slip/m med 2.49 det / 2.59 sto (gate <=2.4), prog med 0.88/0.86 (>=0.8 ok, no falls/terms — NOT a full collapse). DR0 retention intact (gv 12/12, prog 1.05, slip 2.04). Per pre-registration: DR 0.5 is the strafe ceiling for exposure-only training; lateral needs a gait change (mirror-symmetry line, L/R asymmetry already flagged on head90), not more DR. strafe-dr05 remains the lateral robustness rung.

