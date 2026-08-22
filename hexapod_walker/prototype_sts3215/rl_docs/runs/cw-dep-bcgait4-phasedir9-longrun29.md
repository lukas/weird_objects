# cw-dep-bcgait4-phasedir9-longrun29

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T14:16:06+00:00

**pod**: hexapod-mjx-train-0

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-longrun17

**wandb_id**: x4jwj5pn

**hypothesis**: Same recipe as longrun13/longrun17 (fresh re-init from the raw BC clone, --steps 4M, --log-std-anneal-frac 0.3) on a THIRD, previously-untried seed, to find out whether longrun17's just-corrected first-ever rung-A gate PASS (DR-0 det progress 1.02x clone, slip 0.74x, speed 0.069) is a reproducible property of this recipe or another one-off seed draw -- longrun13 (seed13) got WORSE with the identical recipe, so the two existing data points disagree and neither a clean 'budget helps' nor 'budget doesn't help' story is settled. Prediction-if-recipe-works: seed29 also lands near/above the gate (progress >=0.85x clone, slip <=1.2x) -- 2-of-3 would support promoting the 4M/anneal-0.3 schedule as the lineage's actual fix, pending the seed13-vs-seed17 divergence still being explained. Prediction-if-seed-luck: seed29 lands back near longrun13/seed17's original bad 2M level (progress ~0.7-0.8x, slip ~1.3x) or worse -- 1-of-3 confirms this is basin-selection noise, not a repeatable recipe, and closes the budget-scaling question for good in favor of the BC-anchor/phase-lock family-boundary trace.

**gate**: Same clone-relative forward panel as pd9 (logs/ckpt_eval/phasedir3_clone_control_gate, DR-0 det+sto). Read DET-mode DR-0 progress/slip/speed/falls as headline (sto is not part of the ratio criteria -- the clone's own sto baseline is degenerate). PASS = zero falls, gait_valid 6/6, progress >=0.9x clone, slip <=1.15x clone, speed in [0.06,0.096]. Compare directly against longrun13 (0.792x/1.286x, FAIL) and the corrected longrun17 (1.02x/0.74x, PASS partial) before drawing any lineage-wide conclusion; do not verdict until eval_<run>.log shows SYNCED (this cycle's longrun17 verdict was corrected after a premature-verdict race).

