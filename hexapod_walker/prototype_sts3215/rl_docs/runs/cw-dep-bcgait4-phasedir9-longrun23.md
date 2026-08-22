# cw-dep-bcgait4-phasedir9-longrun23

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: finished

**created**: 2026-08-22T14:25:00+00:00

**pod**: hexapod-mjx-train-0

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-longrun17

**wandb_id**: zfpq0jwf

**hypothesis**: Plain English: the same training recipe passed our walking bar on one random seed and failed on another, so we are running two more seeds to measure how often it passes before trusting or promoting it. Context: longrun17 (seed17) is the lineage's first det rung-A PASS (DR-0 progress 1.02x clone, slip 0.74x, speed 0.069) while the identical recipe on seed13 (longrun13) regressed (0.792x/1.286x). The 08-22 per-tick cadence trace (logs/ckpt_eval/pd9seed17_bc_cadence_trace) exonerated the BC-anchor/phase-lock supervision entirely (no cadence gap; supervision tracked 4x tighter than the clone tracks itself), leaving the init/seed-basin lottery on a ~flat reward surface as the root cause of the divergence. With longrun29 (seed29) already running, this seed23 twin makes the sample n=4 completed seeds. Prediction-if-true (recipe is a usable lottery, pass rate >=~50%): seed23 or seed29 lands det progress >=0.9x clone with slip <=1.2x, zero falls, gait 6/6 -- then select-best-of-N seeds + independent re-eval of the passer is the promotion path for rung A. Prediction-if-false: both new seeds land in the 0.7-0.8x/1.2-1.4x regression basin -- pass rate ~1/4, the lottery is too thin to farm, and the next lever is stance-slip pricing that survives the optimization regime, not more seeds. Strongest alternative: eval noise on longrun17's single reading (checked separately by its own queued independent re-eval).

**gate**: At 4M, DR-0, same clone-relative forward panel as the lineage (logs/ckpt_eval/phasedir3_clone_control_gate). PASS = zero falls, gait_valid 6/6, det progress >=0.9x clone AND slip <=1.2x clone (longrun17's own bar). Report exact ratios vs clone AND vs longrun17/longrun13 either way; verdict must wait for the eval SYNCED marker per the 08-22 process note. This arm measures the recipe's seed pass rate -- no promotion decision from this run alone.

