# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-seed13

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-23T06:41:45+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05

**wandb_id**: tad9fo37

**hypothesis**: Plain English: is tipfrac05's FIRST-EVER full M5 pass (turn+push+fault composition, tips 0.162/0.184) a real recipe or a lucky seed? Single lever vs tipfrac05: --seed 13 only (everything else byte-identical: goal.walk_turn_in_place_frac=0.5, same overshoot-decay pricing, same permanent fault/push cfg, same pre-cheat turnfault-seq1 init, same 2M budget). If-true: tip err stays roughly in the same <=0.20-0.25 band and eval_amp_m5 m5_pass stays true or near-true -- the turn-practice-density mechanism is seed-robust, matching the joystick track's own n>=3-4 seed promotion bar. If-false: tip err regresses toward the frac=0/0.2 band (0.25-0.30ish) -- frac=0.5's win was this seed's basin, not the lever, and a wider seed grid (matching stotight45's n=4 precedent) is needed before any promotion claim. NOTE: a concurrent cycle already launched tipfrac05-acq1 (same seed, +6M acquisition budget) testing the BUDGET-erosion axis; this arm tests the ORTHOGONAL seed-lottery axis at matched 2M discovery budget, no overlap.

**gate**: Repro-PASS = tip-left AND tip-right eval_yaw err <=0.20-0.25 AND eval_amp_m5 m5_pass=true or misses by <=1 section narrowly (matching tipfrac03's near-miss pattern) -- recipe looks seed-robust, fund a full n=4 seed grid next. Repro-FAIL = tip err regresses to the pre-lever 0.25-0.30 band -- frac=0.5's result was this seed's basin only, treat as an n=1 existence proof, widen the seed grid before trusting the dose curve's slope.

**verdict**: Confirms the turn-practice mechanism direction on a second seed; full clean m5_pass is a seed-edge effect, not (yet) a robust recipe. Byte-identical to tipfrac05 except --seed 13: eval_yaw tip-left/right err 0.2276/0.2034 -- both clearly inside the M4 PASS-clean band (<=0.20-0.25) and far from the pre-lever baseline (ypfix1's 0.2471/0.2553), so the mechanism DIRECTIONALLY reproduces (a second independent seed also lands well above the no-practice recipe). But eval_amp_m5 m5_pass=false on TWO sections this time (seed7's tipfrac05 missed none): yaw misses the suite's strict <=0.20 bar on both sides by 0.003-0.028 (tighter margin than seed7's 0.162/0.184), and walk narrowly misses its slip bar (4.04 vs <=3.5) -- but that reading comes from only 3 translating episodes at frac=0.5 (12-episode panel, half-ish are dedicated turn-in-place by design), a small-sample median, not necessarily a real regression. Safety floors are IDENTICAL to seed7 and clean: zero falls/terminations anywhere (walk 0/12, push 2/12 det only, fault 2/12 det only), gait_valid 12/12 every section. READ: this is a seed-basin-edge case, the same class of variance the joystick track's phasedir9/longrun17 lineage already established for this reward regime near a gate bar -- not a lottery-reversal (tip err did NOT regress toward the 0.25-0.30 pre-lever band) and not a clean second full pass either. Per the pre-registered gate, doesn't cleanly fall in either Repro-PASS or Repro-FAIL bucket; recorded as INFORMATIVE. Fund a 3rd/4th seed (matching the joystick track's own n=4 promotion precedent) before any promotion claim on this recipe; tipfrac05 (seed7) remains the champion candidate PASS. Evidence: logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_tipfrac05_seed13_{gate,m5}/.

