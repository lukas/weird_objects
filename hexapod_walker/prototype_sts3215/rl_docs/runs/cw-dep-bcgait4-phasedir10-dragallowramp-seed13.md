# cw-dep-bcgait4-phasedir10-dragallowramp-seed13

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-22T15:12:44+00:00

**pod**: hexapod-mjx-train-0

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-longrun23

**hypothesis**: Plain English: teach the walker not to drag its feet, but ease into that rule instead of applying it full-strength from the very first (still-clumsy) step. Context: the phasedir9 seed-lottery dig-in (n=4: seed17 PASS-partial 1.02x/0.74x, seeds 13/23/29 all FAIL 0.74-0.82x progress/1.17-1.30x slip) closed the 'more seeds' lever per its own pre-registration (pass rate 1/4, too thin to farm) and pointed at 'stance-slip pricing that survives the optimization regime' as the next lever. The pd8 regime-gap dig-in already measured WHY the fixed 24mm drag_stance_allow_mm allowance can't do this: at the training action-noise std (0.135, before the log-std anneal converges) the honest clone's own per-stance travel needs an allowance >=48mm to stay untaxed, while the det drag cheat pays zero past 36mm -- so a fixed tight allowance taxes noisy honest exploration far harder than the cheat DURING the exact window (0-1.2M steps) where PPO picks its basin, which the anneal alone cannot fix since it only shrinks the NOISE, not the mismatched PRICING that noise is priced against. NEW mechanism (built+banked this cycle, rl_move/tests/test_drag_allow_ramp.py 6/6): reward.drag_stance_allow_ramp_steps/_ramp_mm anneals the allowance itself from a loose 48mm down to the validated 24mm target over the SAME 1.2M-step window as the existing --log-std-anneal-frac 0.3 schedule, so noisy honest exploration is never over-taxed relative to the cheat, and by the time std has converged the pricing is exactly the already-validated det-calibrated stack. Re-init from the raw BC clone (lineage rule: repairs never continue a cheat-committed checkpoint) on seed13 -- the seed that near-passed at 2M (0.873x) then regressed with more budget (longrun13, 0.792x) under the OLD fixed allowance, making it the most informative seed to re-test under the new pricing. Prediction-if-true: det DR-0 progress >=0.9x clone AND slip <=1.2x clone (the longrun17 bar), zero falls, gait_valid 6/6 -- the allowance mismatch, not seed luck, was gating the pass. Prediction-if-false: seed13 lands back in the same 0.7-0.8x/1.2-1.3x regression basin -- the ramp is refuted as a fix for this specific seed's basin, and the seed-lottery reading stands as the lineage's real ceiling absent a different (non-allowance) pricing lever. Strongest alternative: the seed13/17 divergence is genuinely random (unrelated to allowance-vs-noise timing) and this ramp changes nothing either way -- distinguished by running the SAME ramp on seed23/29 (also queued this cycle) to see if the flip rate rises above the 1/4 baseline.

**gate**: Same clone-relative forward panel as the lineage (logs/ckpt_eval/phasedir3_clone_control_gate, DR-0 det+sto). PASS = zero falls, gait_valid 6/6, det progress >=0.9x clone AND slip <=1.2x clone (longrun17's own bar). Compare directly against this seed's own longrun13 reading (0.792x/1.286x, FAIL under the old fixed allowance) -- any improvement toward or past the bar is the allowance-ramp signal; report exact ratios either way.

**refused_reason**: hexapod-mjx-train-0 code marker 8f356ccf6820d4c6af8cb9fd3456981bbd6322a5 != local HEAD 2a01307eaf190231bf7e2b50439160e49c45d59a. Sync first: snapshot.sh --sync hexapod-mjx-train-0 (and snapshot/commit before that if the tree is dirty).

