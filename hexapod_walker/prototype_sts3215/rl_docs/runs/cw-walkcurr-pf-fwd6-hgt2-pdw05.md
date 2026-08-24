# cw-walkcurr-pf-fwd6-hgt2-pdw05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-24T00:04:31+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-hgt2

**wandb_id**: 725hv284

**hypothesis**: Plain English: hgt2 (tight height-gate dose) sat completely flat because of a bug, not a bad hypothesis -- the safety grace period (2.0s) exactly matched the default park-duty averaging window (2.0s), so the one existing charge that already prices a permanently-airborne leg never got a chance to fire before termination reset it. Single lever vs hgt2: goal.park_duty_window_s 2.0->0.5 (fills 4x before the grace boundary), everything else byte-identical (sigma=11mm/drop=25mm/grace=2.0s, k_park_duty unchanged at 0.08). Bank-proven this cycle (test_walkcurr_pf_hgt_* now parametrized 4 doses incl. this one, 16/16 green: gait/park/stall clearly beat belly_sit, honest gait untaxed, ranking undisturbed). Also directly answers this cycle's OTHER finished arm, hgt1 (loose dose): hgt1's own-cfg gate shows the policy converging on a NEW static tripod-ish pose (3 legs duty~0.97, 3 legs duty~0.03, height_err~41mm) that sits comfortably UNDER hgt1's loose 60mm cutoff -- this arm's TIGHT 25mm cutoff is well below that 41mm settling point, so a de-confounded park_duty charge at the tight cutoff tests whether the SAME escape is blocked once both the threshold is tight enough and the charge can actually fire. Prediction-if-true: walk_freeprog_score crosses toward/past 0, gait_valid>=4/6 on video. Prediction-if-false (same flat/static signature): the confound was real but not sufficient, escalate to a direct minimum-total-foot-contact charge (new mechanism, needs its own scripted twin) before BC-kickstart.

**gate**: Same rung-1 gate as hgt1/hgt2: C-env det fixed-forward panel -- prog_ratio>0 and gait_valid on >=4/6 det episodes with visible forward travel on video, env/walk_freeprog_score leaves [-0.10,-0.05] and trends toward/past 0 by 2M, clip_fraction stays healthy (no crush). PASS = rung-1 lands, move to rung 2. FAIL with reward_park_duty now nonzero but still flat/static = confound fixed but dose insufficient (read jointly with the -pdx15 sibling). FAIL with reward_park_duty still exactly 0 = confound diagnosis was wrong, re-audit.

