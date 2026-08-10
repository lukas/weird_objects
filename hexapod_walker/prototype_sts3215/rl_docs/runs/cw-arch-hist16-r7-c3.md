# cw-arch-hist16-r7-c3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T18:47:00+00:00

**pod**: hexapod-mjx-train-3

**steps**: 40000000

**parent**: cw-arch-hist16-r7-c1

**wandb_id**: pl4j0b2s

**hypothesis**: Plain English: does the 16-frame temporal-arch walker keep closing the slip/economy gap to the deployment-contract champion with more training, or has it plateaued? c1's +40M continuation PASSED and was STILL improving (slip/m det 1.14, within one eval-noise step of the champion's 0.89-1.13 band, up from r7's 1.43) with no stability regression -- one variable vs c1: +40M more steps warm-started from c1's own checkpoint, same recipe, nothing else changed. If-true: slip/m det moves fully inside the champion band (<=1.13) and sto keeps pace (not flat/regressing) -- the architecture line closes the gap by exposure alone. If-false: slip/m plateaus outside the band despite the extra steps -- the gap is structural (contact-pricing/architecture), not a training-budget question, and the line should try a different lever (e.g. reward shaping) before another blind continuation.

**gate**: own-cfg DR0.5 det+sto 6/6 gv, 0 term, prog med >=1.05; slip/m det improves or holds vs c1's 1.14 (not regressing), sto does not regress vs c1's 1.38; DR0 gate det+sto 6/6 gv, 0 term; JOYSTICK GATE (eval_drive DR0.2) 0 falls; video clean six-leg cycling, no flag leg

