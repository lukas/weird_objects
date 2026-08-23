# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-acq1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T06:38:06+00:00

**pod**: hexapod-mjx-train-4

**steps**: 6000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05

**wandb_id**: wlwtd2jb

**hypothesis**: Plain English: the tipfrac05 recipe just became the first checkpoint to pass the whole cross-engine M5 suite at a 2M discovery budget -- does its turn tracking SURVIVE a real acquisition budget, or does more training erode the expensive turning skill back toward richer hold/forward income (this lineage's known failure mode: turnclone-tip50 eroded the raw clone 0.10->0.14+ under 2M RL; cont1 froze tips but leaked yaw elsewhere at +6M)? Single lever = +6M steps warm-started from the tipfrac05 checkpoint itself, config unchanged (50% dedicated turn-episode exposure, overshoot pricing keys ON, permanent fault+push). Prediction-if-true(stable): tips stay <=0.20 and m5_pass holds -> budget-stable M5 candidate. Prediction-if-false: tips regress >0.20 while reward rises = income-competition erosion -> hold/forward repricing (q_20260823T0240Z item b) returns as the funded lever. Strongest alternative: tips drift slightly but stay in 0.20-0.25 band (partial erosion, dose/budget tradeoff).

**gate**: HOLD = post-isolation-fix eval_amp_m5 m5_pass=true with yaw tips <=0.20 both signs, walk section passing on translating episodes, own-cfg DR-0 gait_valid >=11/12 -> recipe is budget-stable, becomes the M5 candidate checkpoint. ERODED = tips >0.20 (or any m5 section newly failing) while reward rose -> exposure only holds at discovery budget; next lever is hold/forward income repricing, NOT more budget. Judge tips from the m5 yaw section (hazards zeroed), never from DR-0 panel medians (tip-episode contamination gotcha).

**verdict**: Result: budget does NOT preserve turn-tracking -- 6M more steps from the tipfrac05 checkpoint ERODED tip accuracy back toward hold/forward income, exactly the pre-registered ERODED branch. Evidence: eval_amp_m5 tip-left/right err regressed 0.162/0.184 (2M parent, clean-pass) -> 0.204/0.269 (m5_pass=false; tip-right badly over the 0.20 bar, tip-left misses by 0.004); walk section also drifted over its own bar (det_slip_med 3.516 vs 3.5, a hair over, consistent direction not noise-canceling); meanwhile training reward rose 110->226/ep across Q1-Q2 then sat flat 226/226/230 for the last three quarters -- reward improved+plateaued while the targeted skill got worse, a genuine reward<->eval divergence, not a flat-reward stall. Safety/composition floors held: own-cfg DR-0 gait_valid 11/12 (6/6 det + 5/6 sto, meets the run's own >=11/12 bar), m5 push section PASS clean, m5 fault section PASS and even improved (gait_valid 9->11/12 vs the 2M parent) with one legitimately carried fault leg. Video: clean six-leg cycling in both walk and fault sections, no dragging/skating, no falls. Why: confirms the run's own hypothesis -- turning costs 4-10x more in current/gyro/roll than hold/forward per the probe_walk_income measurement, so any additional undirected training re-drifts the optimizer toward the cheaper income source; exposure (the tipfrac curriculum lever) only holds the line at discovery budget, it does not fix the underlying price gap. Next: per q_20260823T0240Z item (b), hold/forward income repricing is now CONFIRMED (not just assumed) as the necessary next M4 turn+push lever -- a bank-gated reward build, not a further budget/seed arm on this exact recipe.

