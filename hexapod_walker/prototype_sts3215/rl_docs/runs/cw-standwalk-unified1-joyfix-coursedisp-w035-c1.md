# cw-standwalk-unified1-joyfix-coursedisp-w035-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-29T07:53:37+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-unified1-mix-long-s0

**wandb_id**: zyegx0st

**hypothesis**: Plain English: shrinking the course-displacement pricing window BELOW the gait's half-stride (1.5s -> 0.35s) should finally make the robot's tick-level heading straighter, because the dig-in measured (probe_dir_floor, 08-29) that a clean reference gait under identical conditions (mesh, 100Hz, 0.375deg/tick slew, 0.08 cmd, DR-0) reads per-tick direction_err mean 13.5/med 5.4 deg while this policy reads 60.3/32.4 on its own real eval ticks -- the 25Hz-era ~35deg 'structural sway floor' is cadence-specific (re-measured 31.5 at 25Hz vs 13.7/5.1 at 100Hz primitive/mesh) and does NOT excuse this excess -- and windows >=0.75s provably integrate the zigzag away (6.2deg at every window 0.75-6.0s on the failed lineage) while the sway lives at the ~0.375s half-stride timescale. Same warm start (long-s0 16M PASS), same stack as coursedisp-c1, ONLY window_s changes; sibling w015-c1 doses the window shorter. Bank: test_course_disp_window_semantics.py 22/22 green at 0.35/0.15 (obey>skew/stall/park/wrongway orderings all hold sub-stride). Predict-if-true: DR-0 det walk direction_err_mean_deg drops >=15deg vs long-s0's 55-65 band at 2M, slip/m stays in parent band (displacement pricing, not the velocity tax that blew slip 2.5x in cmdtrack-c1). Predict-if-false: dir_err flat again or slip blows up -> sub-stride displacement pricing closed; the sway fix then belongs to the stage-2 teacher-distillation line, not reward pricing on this lineage. Strongest alternative: the sway is load-bearing for balance -- policy pays the charge and slows down instead of straightening.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY at 2M. PASS-with-delta: reward_walk_course_disp telemetry on >=50% of commanded walk ticks AND DR-0 det walk direction_err_mean_deg drops >=15deg vs long-s0's ~55-65deg band AND slip/m within 1.5x parent band AND gait_valid>=5/6, no new sacrificed leg, session terminations<=6/90 -> escalate winner of w035/w015 to 2-seed acquisition. PASS-no-delta (fires >=50% but dir_err flat): sub-stride window lever closed -- do NOT fund further coursedisp window/dose arms; route the sway problem to stage-2 distillation. FAIL: reward collapse (vs long-s0 matched-step trend), terminations>6/90, or slip/m >1.5x parent band -> mechanism reverts to window=1.5 default.

