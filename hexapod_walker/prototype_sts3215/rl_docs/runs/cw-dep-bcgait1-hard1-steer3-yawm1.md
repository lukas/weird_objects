# cw-dep-bcgait1-hard1-steer3-yawm1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-19T00:12:38+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-dep-bcgait1-hard1-steer2-stagecurric1

**wandb_id**: qon84cv1

**hypothesis**: Stop the joystick direction-switch leg tangle by charging the robot for riding its hip-yaw joints against their hard stops -- the measured precursor of every tangle -- instead of changing what commands it practices. Operator order fb_20260818T152717 lineage: three exposure-side levers each moved the symptom without curing it (steer1-hard20m1 3/24 over_current, steer2-hard20m1-r1 2/24 + slip eroded to 2.2-3.9/m, steer2-blend1 0/24 but a reproducible flip_180 park/freeze stall + slip still 2.5-3.6/m), and probe_dirswitch_tangle measured hard1 spending 1.3-9.3% of post-switch ticks within 2 deg of (and into, -0.65 deg) the yaw hard stop while rot60 sector crossings were exonerated. This arm adds the NEW reward.k_yaw_margin=2.0 / yaw_margin_allow_deg=3.0 term (built+bank-tested this cycle, snapshot d193bf1a: per walk tick each leg inside the 3-deg band pays k scaled by depth; honest tall gait rides 10-20+ deg and pays ~0, bank-proven) on top of the proven staged-curriculum recipe (stagecurric1's exact cfg, same hard1 warm start, seed 12). Prediction-if-true: panel keeps 0/24 over_current, the flip_180 park/freeze stall disappears (the pinned pose now loses money every tick), slip/m moves back toward hard1's 1.3-1.5 band, and probe yaw_sat_frac drops well under the parent's 0.013-0.093. Prediction-if-false: stall/tangle persists even with yaw_sat_frac driven low -- the precursor theory is wrong; report to operator, no dose sweep.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY (2M): PASS requires (1) finite losses, no sustained KL-rollback storm; (2) periodic eval + final video keep in-band tall height and six-leg cycling (no re-crouch, no permanent leg-sacrifice); (3) retention per operator admission bars (fb_20260818T152717 item 7): original fixed-command hard1 panel stays height >= -20mm, six-leg gait-valid, zero falls, slip <= 1.8/m, comparable progress. INFORMATIONAL (folded into the same-cycle verdict): run the identical 24-episode direction-switch panel (6 families x 2 seeds x det+sto, DR-0 + own-DR-0.35) FORCING goal.walk_cmd_stage=2.0 at eval (this cfg pins stage=0 pre-schedule -- blend1 triage trap) and compare stagecurric1's canary read (0/24 over_current, 1/24 flip_180 park/freeze) + blend1's slip band (2.48-3.55/m): PASS-shaped = 0/24 over_current AND no park/freeze stall AND panel slip medians < 2.0/m AND probe_dirswitch_tangle yaw_sat_frac < 0.01 on this checkpoint -> queue the ~20M hardening continuation with --best-ckpt retention guard; worse on any bar = STOP and report to operator (4th lever on this sub-line; no autonomous 5th).

