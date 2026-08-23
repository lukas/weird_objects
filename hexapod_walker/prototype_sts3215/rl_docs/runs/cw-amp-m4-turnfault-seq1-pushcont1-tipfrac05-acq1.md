# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-acq1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T06:38:06+00:00

**pod**: hexapod-mjx-train-4

**steps**: 6000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05

**wandb_id**: wlwtd2jb

**hypothesis**: Plain English: the tipfrac05 recipe just became the first checkpoint to pass the whole cross-engine M5 suite at a 2M discovery budget -- does its turn tracking SURVIVE a real acquisition budget, or does more training erode the expensive turning skill back toward richer hold/forward income (this lineage's known failure mode: turnclone-tip50 eroded the raw clone 0.10->0.14+ under 2M RL; cont1 froze tips but leaked yaw elsewhere at +6M)? Single lever = +6M steps warm-started from the tipfrac05 checkpoint itself, config unchanged (50% dedicated turn-episode exposure, overshoot pricing keys ON, permanent fault+push). Prediction-if-true(stable): tips stay <=0.20 and m5_pass holds -> budget-stable M5 candidate. Prediction-if-false: tips regress >0.20 while reward rises = income-competition erosion -> hold/forward repricing (q_20260823T0240Z item b) returns as the funded lever. Strongest alternative: tips drift slightly but stay in 0.20-0.25 band (partial erosion, dose/budget tradeoff).

**gate**: HOLD = post-isolation-fix eval_amp_m5 m5_pass=true with yaw tips <=0.20 both signs, walk section passing on translating episodes, own-cfg DR-0 gait_valid >=11/12 -> recipe is budget-stable, becomes the M5 candidate checkpoint. ERODED = tips >0.20 (or any m5 section newly failing) while reward rose -> exposure only holds at discovery budget; next lever is hold/forward income repricing, NOT more budget. Judge tips from the m5 yaw section (hazards zeroed), never from DR-0 panel medians (tip-episode contamination gotcha).

