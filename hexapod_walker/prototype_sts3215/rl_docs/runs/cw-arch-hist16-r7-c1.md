# cw-arch-hist16-r7-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T14:56:14+00:00

**pod**: hexapod-mjx-train-2

**steps**: 40000000

**parent**: cw-arch-hist16-r7

**hypothesis**: OPERATOR-ORDERED CONTINUATION (08-10, fired by the operator assistant during the launch hold): keep training the 16-frame temporal-arch walker from the cw-arch-hist16-r7 PASS checkpoint (md5 32269a363d8dc8e4b9dbb2d9faabca05). External review 08-10: the capability lines match the deployment-contract champion on stability (0 falls, prog at/above champion band) but not economy (slip/m 1.3-1.6 vs 0.89-1.13, ~40 pct worse); that slip gap is judged against contact physics the tape session showed sim mismeasures, so the ruling is keep training the line now and re-baseline slip caps after the actuator/contact calibration lands. Identical config to r7, warm start, +40M steps (auto-continue segment convention). If-true: r7 gates still pass with no regression and slip/prog drift toward the champion band. If-false: plateau at r7 levels, meaning more steps do not buy economy under current contact pricing and the line waits for recalibration.

**gate**: det gait_valid 6/6 own-cfg DR0.5 + JOYSTICK GATE (eval_drive DR0.2) zero in-envelope falls + det prog_ratio med >=0.85 AND no regression vs r7 (prog med around 1.0-1.17 band, terms 0); slip/m REPORTED vs r7 band 1.3-1.6 but NOT gated (slip caps await contact recalibration per operator 08-10); frames watched det

