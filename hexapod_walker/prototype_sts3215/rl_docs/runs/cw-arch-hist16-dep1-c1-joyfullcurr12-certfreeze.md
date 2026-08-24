# cw-arch-hist16-dep1-c1-joyfullcurr12-certfreeze

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-24T13:20:13+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr9-stopcur2

**wandb_id**: ffhhv474

**hypothesis**: Plain English: the robot got worse at not-falling because we forced its stop-hold during TRAINING, which corrupts the learning data (the action it proposes on frozen ticks is never executed, so it can drift unpunished and then jerk at resume); this run keeps the hold ONLY in the certification assay -- where it legitimately unlocked the never-practiced side/rear buckets -- and trains on clean on-policy data. Warm-start from stopcur2 (the clean 1/48-joygate parent), identical reward cfg (k_walk_stop_current=2.0), goal.walk_stop_freeze_s=0.0 in training, new --walkcurr-cert-cfg-set goal.walk_stop_freeze_s=0.4 applied to the cert/precert env only (default-off flag, 24/24 tests, tag exp/cw-arch-hist16-dep1-c1-joyfullcurr12-certfreeze). freeze40's dig-in 2x2 (08-24): eval-time freeze on parent weights costs ~1 fall (1/48->2/48) while training-time freeze cost the WEIGHTS 1/48->4/48 (freeze-off eval; same det episodes fall either way) and 7/48 combined -- so removing it from training should keep the promotion win without the safety regression.

**gate**: walkcurr/frontier promotes past b1 (>=b2) at/near init via the freeze-assisted cert (precert b1 PASS expected at step 0) and b2+ get real practice; held-out 60s joygate stays in the parent band: falls <=2/48 (parent 1/48, freeze40 7/48), dir_err med dr0 <=~36deg (parent 33.9), slip med <=2.9, no new leg-sacrifice signature. NOTE for triage: the joygate prestage forwards training cfg, so it will correctly eval freeze-OFF. If-true: on-policy corruption confirmed as freeze40's damage mechanism; cert-only freeze becomes the standard ladder assist. If-false (joygate still regresses to ~4+/48 with training freeze off): the damage is the b2+ practice diet itself trading off stress-mix robustness -- next lever is mixing joygate-style stress_mix commands into bucket training, not freeze mechanics.

