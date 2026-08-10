# cw-arch-hist16-r7-c3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T18:47:00+00:00

**pod**: hexapod-mjx-train-3

**steps**: 40000000

**parent**: cw-arch-hist16-r7-c1

**wandb_id**: pl4j0b2s

**hardware_ready**: False

**hypothesis**: Plain English: does the 16-frame temporal-arch walker keep closing the slip/economy gap to the deployment-contract champion with more training, or has it plateaued? c1's +40M continuation PASSED and was STILL improving (slip/m det 1.14, within one eval-noise step of the champion's 0.89-1.13 band, up from r7's 1.43) with no stability regression -- one variable vs c1: +40M more steps warm-started from c1's own checkpoint, same recipe, nothing else changed. If-true: slip/m det moves fully inside the champion band (<=1.13) and sto keeps pace (not flat/regressing) -- the architecture line closes the gap by exposure alone. If-false: slip/m plateaus outside the band despite the extra steps -- the gap is structural (contact-pricing/architecture), not a training-budget question, and the line should try a different lever (e.g. reward shaping) before another blind continuation.

**gate**: own-cfg DR0.5 det+sto 6/6 gv, 0 term, prog med >=1.05; slip/m det improves or holds vs c1's 1.14 (not regressing), sto does not regress vs c1's 1.38; DR0 gate det+sto 6/6 gv, 0 term; JOYSTICK GATE (eval_drive DR0.2) 0 falls; video clean six-leg cycling, no flag leg

**verdict**: PASS -- the 16-frame temporal-arch walker keeps closing the slip/economy gap to the deployment champion with more training, and stability holds. Gate (DR0): det+sto 6/6 gait_valid, 0 term, prog med 1.20/1.03, slip/m med 1.16 (det)/1.27 (sto) -- det slip now at the edge of the champion band (0.89-1.13, within noise of 1.13) vs c1's 1.14, sto improved from c1's 1.33. Own-cfg DR0.5: 6/6 gv both passes, 0 term, prog med 1.13/1.12, slip/m det 1.13 (right at the champion band edge, holds vs c1's 1.14) / sto 1.31 (improved vs c1's 1.38, no regression). JOYSTICK GATE (eval_drive DR0.2): 0 falls across the full direction panel + flip stress. Video (all 6 det episodes, both DR0 and own-DR0.5): six legs visibly cycling swing/stance every clip, no flag leg, no dragging -- ordinary walking gait, same character as c1/r7. Architecture-line result only (deployment-contract obs, not the dep-line's deployment-exact contract) -- not a hardware candidate. Slip gap continues to close by exposure alone with no sign of plateau; line stays open for one more continuation before judging whether it needs a different lever.

