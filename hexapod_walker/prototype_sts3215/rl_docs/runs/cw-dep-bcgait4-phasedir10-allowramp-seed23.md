# cw-dep-bcgait4-phasedir10-allowramp-seed23

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-22T15:32:53+00:00

**pod**: hexapod-mjx-train-0

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-longrun23

**hypothesis**: Plain English: does the drag-allowance-ramp fix (which measurably helped seed13) also help a DIFFERENT failing seed, or was seed13 a one-off? Context: cw-dep-bcgait4-phasedir10-allowramp-a (seed13, a concurrent cycle's arm, verdict pending my triage this cycle) applied reward.drag_stance_allow_ramp_steps=1.2M/ramp_mm=48 (bank-tested this cycle, rl_move/tests/test_drag_allow_ramp.py 6/6) on top of the identical phasedir9/longrun13 recipe and moved BOTH clone-relative progress (0.792x->0.831x) and slip (1.286x->1.159x) toward the gate simultaneously -- the first time in the whole phasedir1-10 lineage a single lever improved both axes at once -- though it still missed the 0.9x/1.2x pass caps. This arm applies the SAME unchanged ramp to seed23 (my own verdict this cycle: longrun23 FAIL 0.818x progress/1.175x slip under the OLD fixed 24mm allowance) to test generalization on a second, independent seed. Prediction-if-general: seed23 improves on both axes vs its own longrun23 baseline by a comparable or larger margin (progress +0.03-0.04, slip -0.1 to -0.15) -- two-for-two supports arming the ramp as a lineage default and motivates a slower/longer ramp or more budget as the next lever, not abandoning it. Prediction-if-seed13-specific: seed23 stays flat/worse -- seed13's improvement was basin-specific luck, and the regime-gap-ramp mechanism itself (not just its current dose/schedule) is refuted; return to the BC-anchor/phase-lock residual-slip dig-in. Strongest alternative: seed29 (concurrent cycle's own generalization arm, the worst-performing seed in the n=4 baseline) is a harder test than this one; read seed23 and seed29 together, not either alone.

**gate**: Same clone-relative forward panel (logs/ckpt_eval/phasedir3_clone_control_gate, DR-0 det+sto). PASS = zero falls, gait_valid 6/6, progress >=0.9x clone, slip <=1.2x clone, speed in [0.06,0.096]. Report exact ratios vs this seed's own longrun23 baseline (0.818x/1.175x) either way; a two-of-two (this + seed29) improvement pattern is the promotion signal, not this run alone.

