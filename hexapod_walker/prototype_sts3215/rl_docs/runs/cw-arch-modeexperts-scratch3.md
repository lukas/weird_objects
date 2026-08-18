# cw-arch-modeexperts-scratch3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-17T15:18:00+00:00

**pod**: hexapod-mjx-train-2

**steps**: 40000000

**parent**: cw-arch-modeexperts-scratch2

**wandb_id**: puvo5i2y

**hypothesis**: Give the four rise/hold/lower/loco experts a second, rebalanced training stage so the two lagging skills (walking, sitting-down) catch up to stand-up's cumulative practice, and see whether the isolated-expert architecture can push all three real skills (rise/loco/lower) over our strict success bar once each has gotten a fair budget. Scratch2 (its first 40M) proved the mechanism keeps learning for real (per-expert stds and in-loop eval trends diverge and improve, no cheat in any of the 4 skill videos) but also measured that hold soaked up 2x its intended practice share (.21 realized vs .15 cap) while walking and sitting-down stayed under their .30 target (.24 and .24) -- because sequence episodes reach the hold segment more often now that stand-up reliably completes, so hold banks ticks a from-scratch policy doesn't need as much anymore. This stage cuts mode_seq .20->.10 (fewer full sequences means less of that hold-segment leakage) and re-solves the single-mode mix from the measured realized/commanded ratios (rise .303->.254, walk .345->.383, lower .324->.352, hold .028->.011) so cumulative lineage active ticks should clear ~20M each for rise/loco/lower by the end of this stage (were 13.2M/10.0M/10.1M including the canary). If a skill is still flat at that exposure, that indicts its own reward/start curriculum, not capacity or cross-expert interference (the isolation is architectural, gradients cannot cross).

**gate**: ACQUISITION stage cont'd (pre-registered, MODE_EXPERTS_DIRECTIVE.md SCRATCH3): (a) completes 40M with no NaN/crash -- one retry from latest periodic ckpt on silent death, infra not science; (b) cumulative lineage active_ticks (canary+scratch2+scratch3) reach >=20M each for rise/loco/lower by the end -- report the arithmetic explicitly, a shortfall re-solves scratch4's mix rather than killing; (c) per-expert learning signal stays visible (independent stds, per-mode eval trends) -- flat/degenerate stds on any expert is an implementation-bug trigger, not a skill verdict. Skill SUCCESS verdict (bulk harness det+sto, DR0+own-DR) is now due at THIS fork: >=1/6 success on a skill that scored 0/6 in scratch2 counts as real progress; a skill still 0/6 at >=20M cumulative active ticks is a reward/curriculum bottleneck per the pre-registered decision table, not a capacity failure. NO early stop for poor skill mid-run -- stop only for NaN/crash, a proven exploit dominating a milestone video, or clause (c) instrumentation dying.

