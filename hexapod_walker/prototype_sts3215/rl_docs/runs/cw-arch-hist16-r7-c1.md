# cw-arch-hist16-r7-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T14:56:14+00:00

**pod**: hexapod-mjx-train-2

**steps**: 40000000

**parent**: cw-arch-hist16-r7

**wandb_id**: ehwaukw6

**hardware_ready**: False

**hypothesis**: OPERATOR-ORDERED CONTINUATION (08-10, fired by the operator assistant during the launch hold): keep training the 16-frame temporal-arch walker from the cw-arch-hist16-r7 PASS checkpoint (md5 32269a363d8dc8e4b9dbb2d9faabca05). External review 08-10: the capability lines match the deployment-contract champion on stability (0 falls, prog at/above champion band) but not economy (slip/m 1.3-1.6 vs 0.89-1.13, ~40 pct worse); that slip gap is judged against contact physics the tape session showed sim mismeasures, so the ruling is keep training the line now and re-baseline slip caps after the actuator/contact calibration lands. Identical config to r7, warm start, +40M steps (auto-continue segment convention). If-true: r7 gates still pass with no regression and slip/prog drift toward the champion band. If-false: plateau at r7 levels, meaning more steps do not buy economy under current contact pricing and the line waits for recalibration.

**gate**: det gait_valid 6/6 own-cfg DR0.5 + JOYSTICK GATE (eval_drive DR0.2) zero in-envelope falls + det prog_ratio med >=0.85 AND no regression vs r7 (prog med around 1.0-1.17 band, terms 0); slip/m REPORTED vs r7 band 1.3-1.6 but NOT gated (slip caps await contact recalibration per operator 08-10); frames watched det

**verdict**: PASS -- confirms if-true. +40M steps on the identical config move the 16-frame temporal-arch walker toward the deployment-contract champion's economy with no regression on stability. DR0 gate: det+sto 6/6 gait_valid, 0 term, 0 sacrificed legs, prog med 1.21/1.03; slip/m med improved to 1.14 (det) / 1.33 (sto) vs r7's own 1.43/1.40 -- moving toward the champion band (0.89-1.13) as hypothesized, not plateaued. Own-cfg DR0.5: det+sto 6/6 gv, 0 term, prog med 1.13/1.08, slip/m med 1.16 (det, now within one eval-noise step of the champion band)/1.38 (sto, flat vs r7's 1.37 -- no regression). JOYSTICK GATE (eval_drive DR0.2, heading 45deg/speed 0.06): 0 falls across the full direction panel + flip stress, same clean result as r7. Video (both DR0 and own-DR0.5 contact sheets, all 6 det episodes each): six legs visibly cycling swing/stance every episode, no flag leg, no dragging -- an ordinary, if unremarkable, walking gait, same character as r7. Architecture-line result only (deployment-contract obs, not the dep-line's deployment-exact contract) -- not a hardware candidate. Line stays open per operator directive (1-2 pods); the slip gap vs the champion is judged against contact pricing known to be miscalibrated (open problem 1), not gated here.

