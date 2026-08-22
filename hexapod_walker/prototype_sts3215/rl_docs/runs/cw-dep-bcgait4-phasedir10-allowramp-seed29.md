# cw-dep-bcgait4-phasedir10-allowramp-seed29

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T15:26:03+00:00

**pod**: hexapod-mjx-train-2

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-longrun29

**wandb_id**: nvqwetfc

**hypothesis**: Cross-seed generalization check for the drag-stance-allowance-ramp regime-gap fix: on seed13, the identical single-change A/B (respec of longrun13 adding reward.drag_stance_allow_ramp_steps=1.2M/ramp_mm=48) moved BOTH clone-relative progress (0.792x->0.830x) and slip (1.284x->1.162x) toward the gate simultaneously for the first time in the lineage, though it still missed the 0.9x/1.15x caps (see cw-dep-bcgait4-phasedir10-allowramp-a verdict). This arm applies the SAME ramp, unchanged, to seed29 -- the worst performer in the n=4 seed sample (longrun29: 0.740x progress / 1.296x slip, both clearly FAIL) -- to find out whether the regime-gap fix is a general pricing repair (helps every seed, even the worst) or an artifact specific to seed13's basin.,

**gate**: Same clone-relative forward panel (logs/ckpt_eval/phasedir3_clone_control_gate, DR-0 det+sto). PASS = zero falls, gait_valid 6/6, progress >=0.9x clone, slip <=1.15x clone, speed in [0.06,0.096]. Prediction-if-general-fix: seed29 improves on BOTH axes vs its own longrun29 baseline (0.740x/1.296x), by a margin comparable to seed13's (+0.038 progress, -0.122 slip) or larger -- supports arming the ramp as a lineage-wide default and motivates a slower-ramp follow-up before promotion. Prediction-if-seed13-specific: seed29 stays flat or gets worse -- the seed13 result was basin-specific luck, not a general pricing fix; abandon the ramp mechanism and return to the BC-anchor/phase-lock residual-slip dig-in instead.

