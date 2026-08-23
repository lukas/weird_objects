# cw-walkcurr-pf-fwd6-hgt2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T23:33:51+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-rscale50

**wandb_id**: h9xgpw7y

**hypothesis**: Plain English: dose sibling of hgt1 (running) -- same two mechanisms (walk_height_gate income-gating + safety.walk_max_height_drop_mm early termination) closing the belly-sit escape hatch, but at the TIGHT dose already proven elsewhere (WALKCURR4 lineage, 08-18: sigma=11mm, drop=25mm, grace=2.0s) instead of hgt1's loose from-scratch-friendly dose (sigma=15mm, drop=60mm, grace=1.5s). Tests the dose axis directly instead of waiting for hgt1 to read before trying a second dose (operator 08-22 batching ruling): does the tighter, hardware-precedented pricing suppress the collapse MORE decisively (bank-measured this cycle: tight-dose belly_sit terminates at the 2.0s grace boundary, ~50 steps, vs hgt1's ~43), or does it instead choke early clumsy from-scratch exploration (the exact loadslip-bootstrap-at-full-dose failure mode -- too-tight pricing froze discovery on a DIFFERENT lever this same track already refuted once)? Both doses pass the identical test_walkcurr_pf_hgt_* bank (now parametrized 2x, 8/8 green: gait/park/stall all clearly beat belly_sit, honest gait keeps >=90% of ungated income, v2e ranking undisturbed, early termination fires) -- this arm is the SAME mechanism at a different, independently bank-legal calibration point, not a new mechanism needing its own bank. Prediction-if-true (tight beats loose): walk_freeprog_score crosses 0 faster/further than hgt1 by 2M. Prediction-if-tight-fails-loose-succeeds: pricing too tight for from-scratch discovery, adopt hgt1's looser dose as the mechanism's operating point. Prediction-if-both-fail identically (same belly-sit signature persists): the height-gate+termination mechanism class is insufficient regardless of dose, escalate to a direct minimum-total-foot-contact charge.

**gate**: Same rung-1 gate as hgt1: C-env det fixed-forward panel -- prog_ratio>0 and gait_valid on >=4/6 det episodes with visible forward travel on video, env/walk_freeprog_score leaves [-0.10,-0.05] and trends toward/past 0 by 2M, clip_fraction stays healthy. Read jointly with hgt1: whichever dose (or neither) crosses into real stepping decides the mechanism's operating point before any rung-2 respec.

