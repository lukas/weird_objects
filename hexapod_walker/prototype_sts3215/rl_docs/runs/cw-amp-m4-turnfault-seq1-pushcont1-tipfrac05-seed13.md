# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-seed13

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T06:41:45+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05

**wandb_id**: tad9fo37

**hypothesis**: Plain English: is tipfrac05's FIRST-EVER full M5 pass (turn+push+fault composition, tips 0.162/0.184) a real recipe or a lucky seed? Single lever vs tipfrac05: --seed 13 only (everything else byte-identical: goal.walk_turn_in_place_frac=0.5, same overshoot-decay pricing, same permanent fault/push cfg, same pre-cheat turnfault-seq1 init, same 2M budget). If-true: tip err stays roughly in the same <=0.20-0.25 band and eval_amp_m5 m5_pass stays true or near-true -- the turn-practice-density mechanism is seed-robust, matching the joystick track's own n>=3-4 seed promotion bar. If-false: tip err regresses toward the frac=0/0.2 band (0.25-0.30ish) -- frac=0.5's win was this seed's basin, not the lever, and a wider seed grid (matching stotight45's n=4 precedent) is needed before any promotion claim. NOTE: a concurrent cycle already launched tipfrac05-acq1 (same seed, +6M acquisition budget) testing the BUDGET-erosion axis; this arm tests the ORTHOGONAL seed-lottery axis at matched 2M discovery budget, no overlap.

**gate**: Repro-PASS = tip-left AND tip-right eval_yaw err <=0.20-0.25 AND eval_amp_m5 m5_pass=true or misses by <=1 section narrowly (matching tipfrac03's near-miss pattern) -- recipe looks seed-robust, fund a full n=4 seed grid next. Repro-FAIL = tip err regresses to the pre-lever 0.25-0.30 band -- frac=0.5's result was this seed's basin only, treat as an n=1 existence proof, widen the seed grid before trusting the dose curve's slope.

