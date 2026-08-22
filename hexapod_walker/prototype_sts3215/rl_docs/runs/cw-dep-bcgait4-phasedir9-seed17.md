# cw-dep-bcgait4-phasedir9-seed17

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-22T12:49:14+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-dep-bcgait4-phasedir9-stdanneal

**wandb_id**: h9nrxse9

**hardware_ready**: False

**hypothesis**: phasedir9-stdanneal (fresh BC-clone restart + forced log_std anneal 0.135->0.041) was the best arm of the whole phasedir lineage (progress 0.873x clone, near-PASS, zero falls 24/24) but its own continuation (-cont1, +4M steps) came out WORSE on every clone-relative axis (progress 0.655-0.706x, slip 1.59-1.74x, speed below the 0.06 floor) despite reward partially recovering after a mid-run collapse -- an anomaly now flagged DIG-IN rather than snap-verdicted. Before spending more budget on that disputed continuation, this arm asks a simpler, orthogonal question: is pd9's 2M near-pass itself reproducible, or was it seed luck? Identical config (pd8 reward stack + --log-std-final -3.2/--log-std-anneal-frac 0.6 + overspeed ref floor), same 2M budget, ONLY --seed changed 13->17. Prediction-if-true (reproducible): a second seed also lands zero-falls/gait-6-6 with progress in the same 0.80-0.95x clone range -- pd9's recipe is a real, repeatable near-pass and the lineage is worth the cont1 dig-in's time. Prediction-if-false: seed17 lands far below pd9 (e.g. back near pd8's 0.77x or worse) or shows a different failure mode -- pd9 was partly seed luck and the dig-in must weight that in before trusting any single continuation's regression or progress as signal.

**gate**: At 2M, DR-0, same clone-relative forward panel as pd9 (logs/ckpt_eval/phasedir3_clone_control_gate). Report vs pd9-stdanneal's own numbers (progress 0.873x, slip 1.08x, speed 0.06-0.063, zero falls) on every axis, not just PASS/FAIL against the 0.9x cap. PASS-reproduced = zero falls, gait_valid 6/6, progress within ~0.1x of pd9's 0.873x (i.e. >=0.75x clone), slip <=1.2x clone. Either way this is evidence for the pd9-cont1 dig-in, not a standalone lineage promotion decision -- do not respec a 3rd continuation from this arm without the dig-in's root cause first.

**verdict**: FAIL vs the 0.9x cap; does NOT reproduce pd9's near-pass reading -- seed variance is real and substantial. Same recipe as phasedir9-stdanneal (pd8 stack + std anneal 0.135->0.041 + overspeed ref floor), ONLY seed changed 13->17, same 2M budget. Clone-relative det: progress 0.736x (DR0) / 0.732x (ownDR) -- well below pd9's 0.873x and below even the 0.75x reproducibility bar this run's own gate set; slip 1.273x/1.509x -- OVER the 1.15x cap (pd9 was 1.08x PASS); speed 0.053/0.0565 -- BELOW the 0.06 floor (pd9 was 0.06-0.063 in-band). dir_err actually better than clone (-2 to +1.7deg). Zero falls, gait_valid 6/6 both passes -- the gait itself stays clean and stable, only the clone-relative magnitude regresses. READING: pd9's 0.873x was likely on the favorable end of this recipe's seed distribution, not a reliably repeatable near-pass -- seed17 lands closer to phasedir8's own territory (0.770x) than to pd9's. Combined with pd9-cont1's FAIL (continuing pd9's own checkpoint also regressed, via the init-basin-flatness mechanism), this lineage's ceiling under the CURRENT reward stack looks like it sits in the 0.73-0.87x clone band across inits/continuations, not reliably above the 0.9x cap. Per this run's own gate text: do not respec a 3rd continuation off pd9's checkpoint without root-causing first. NEXT: either (a) run 1-2 more seeds to map the distribution properly before concluding the recipe's ceiling, or (b) treat 0.9x as needing a genuinely new lever (the BC-anchor/phase-lock family boundary dig, pd8 branch (ii), never tried) rather than more seeds/continuations of the same stack. NO DOWNLOAD_ANSWER change.

