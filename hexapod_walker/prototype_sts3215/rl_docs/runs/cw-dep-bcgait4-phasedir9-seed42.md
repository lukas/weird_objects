# cw-dep-bcgait4-phasedir9-seed42

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-22T13:16:22+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-dep-bcgait4-phasedir9-stdanneal

**wandb_id**: dsl8xgda

**hypothesis**: Third data point mapping the phasedir9-stdanneal recipe's seed distribution (pd8 stack + forced std anneal 0.135->0.041 + overspeed ref floor, from the raw BC clone, no reward change). Seed 13 (pd9 itself) landed progress 0.873x clone (near-pass, best of lineage). Seed 17 landed 0.73-0.74x clone (slip/speed also missed their bands) -- i.e. NOT a repeat near-pass, real seed variance. This arm (seed 42) is the tie-breaker: with n=3 seeds we can tell whether 0.873x was the favorable tail of a distribution centered lower (seed17-like is typical, pd9 was luck) or whether these two together already bracket a real spread worth exploiting (run several seeds, keep the best). Same 2M budget, zero reward/env changes -- purely a seed-variance measurement, the most boring and most informative next step available on this lineage per the cycle discipline.

**gate**: At 2M, DR-0 det, same clone-relative forward panel (logs/ckpt_eval/phasedir3_clone_control_gate). Report progress/slip/speed/dir_err vs BOTH pd9 (0.873x/1.08x/0.06-0.063/-5deg) and seed17 (0.73x/1.3-1.5x/0.053-0.057/-2 to +2deg) explicitly -- this is a 3-point distribution reading, not a standalone PASS/FAIL. If this seed also lands 0.75-0.80x range, treat pd9 as the outlier and stop seed-hunting this exact recipe (move to the BC-anchor/phase-lock family boundary lever instead). If it lands >=0.85x, seed-hunting this recipe (run 2-3 more, keep-the-best) becomes a reasonable lever and should be queued.

**verdict**: Stale triage leak, closed on W&B evidence: this -3.2-era phasedir9-stdanneal seed-distribution arm (seed42, 2M steps) never learned — ep_rew_mean ended at -116 with quarters -96/-390/-485/-331, reward declining most of the run (genuine FAIL per RUN_INTERPRETATION_RULES: nothing learning with adequate budget; this matches the known 1/4-pass seed-lottery pathology of the -3.2 dose). No eval artifacts were ever produced and none are warranted for a reward-collapsed basin. The question it fed is superseded: the -4.5 noise floor (stotight45) converted the seed lottery into a 4/4 reproducible recipe (seeds 13/17/23/29), and per-seed dose bests are now fully mapped. No follow-up.

