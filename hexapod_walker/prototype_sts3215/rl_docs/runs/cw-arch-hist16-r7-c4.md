# cw-arch-hist16-r7-c4

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T22:11:22+00:00

**pod**: hexapod-mjx-train-3

**steps**: 40000000

**parent**: cw-arch-hist16-r7-c3

**hypothesis**: Plain English: keep training the 16-frame architecture walker to see if the slip/economy gap to the deployment champion keeps closing or has plateaued. cw-arch-hist16-r7-c3's +40M continuation PASSED and was still improving (det slip/m 1.16, sto 1.27, both closer to the champion band 0.89-1.13 than c1's 1.14/1.33) with zero stability regression and a clean joystick gate. One variable vs c3: +40M more steps, same recipe, nothing else changed. If-true: slip/m moves fully inside the champion band and sto keeps closing -- the architecture line closes the gap by exposure alone, no reward change ever needed. If-false: slip/m plateaus outside the band despite the extra steps -- the gap is structural (contact-pricing/architecture), not a training-budget question, and the line should try a different lever before another blind continuation.

**gate**: own-cfg DR0.5 det+sto 6/6 gv, 0 term, prog med >=1.0; slip/m det improves or holds vs c3's 1.13 (not regressing), sto improves or holds vs c3's 1.31; DR0 gate det+sto 6/6 gv, 0 term; JOYSTICK GATE (eval_drive DR0.2) 0 falls; video clean six-leg cycling, no flag leg

