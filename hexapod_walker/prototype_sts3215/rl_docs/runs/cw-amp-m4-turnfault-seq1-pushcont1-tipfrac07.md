# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac07

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T06:50:50+00:00

**pod**: hexapod-mjx-train-7

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05

**wandb_id**: xo64xo0s

**hypothesis**: Plain English: the turn-exposure dose-response is monotonic through 0.2/0.3/0.5 (tips 0.207/0.234 -> 0.201/0.214 -> 0.162/0.184) -- does pushing dedicated turn-episode exposure to 0.7 keep improving turn accuracy, or does starving translation exposure (only 30% of episodes left for walking) start eroding the walk section? Single lever vs tipfrac05: goal.walk_turn_in_place_frac 0.5->0.7, everything else matched (same pre-cheat init, 2M, pricing keys, permanent fault+push). Prediction-if-true(better): tips < 0.162/0.184 with walk section still clean -> curve still rising. Prediction-if-false(turnover): tips flat/worse or walk erodes -> 0.5 is the sweet spot, dose curve closed. Strongest alternative: tips improve but walk-section translating-episode slip/terms degrade (trade, not free win).

**gate**: BETTER = m5 yaw tips improve on tipfrac05's 0.162/0.184 AND walk section still passes post-fix eval_amp_m5 (translating-episode det slip <=3.5, terms 0, gait_valid 12/12) -> 0.7 becomes the recipe dose. TURNOVER = tips flat/worse OR walk section erodes -> dose curve closed at 0.5. Judge tips from the m5 yaw section (hazards zeroed), walk from translating episodes only (contamination gotcha).

