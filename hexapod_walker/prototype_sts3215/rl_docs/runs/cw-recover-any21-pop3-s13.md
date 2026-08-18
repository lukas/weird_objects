# cw-recover-any21-pop3-s13

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-18T00:49:46+00:00

**pod**: hexapod-mjx-train-3

**steps**: 40000000

**parent**: cw-recover-any19-pop3-s13

**wandb_id**: fe8501ac

**hardware_ready**: False

**hypothesis**: Teach the robot to stand back up from any fallen position by racing three fresh identical-recipe seeds in lockstep and always adopting the first retention-clean winner at each curriculum bucket; this cohort (member 2, seed 13, of population recover-any21-pop3, roster s11,s12,s13, predeclared W&B ids f14d9993,a705c488,fe8501ac, this member fe8501ac) completes the any21 roster after s11 (RUNNING, waiting at its B0 barrier) — any20 was lost to a pure orchestration race (member 2 stopped 20s before a successful s11/s12 repair), not a code defect; this is the operator-preauthorized fallback fb_20260818T002830_3d14e2, re-synced to current HEAD after a code-marker race on the first s12 attempt. From scratch, absolutely NO --init-from. Never reuse any16-any20 names.

**gate**: Live integration gate (inherited from operator fb_20260818T001206_0ee733, roster per fallback fb_20260818T002830_3d14e2): (1) commands at 8fbb7b2 or descendant, exact names/seeds, id roster f14d9993/a705c488/fe8501ac exact on every member, no init_from; (2) all 3 stop at exactly 655,360 with valid ready_B00; (3) leader publishes start_B00 once roster complete, all 3 cross 655,360; (4) first cert on all members CERT/recover_training_envs_synchronized=512; (5) exactly one B1 winner, all 3 ACK identical hash; (6) release_B01 only after all 3 ACKs; (7) all 3 resume from B1 and race B2. Fail closed + preserve evidence; never silently continue a partial cohort.

**verdict**: Weakest of the any21-pop3 cohort but still a real, non-exploit result. Population sync PROVEN live at FULL 40M budget (synchronized=512 every round, adoptions B1-B14 logged, no init-from). Frontier reached B14 (tangle_deep) by 16M and held through 40M (shared cohort history through the last promotion). Matched one-shot gate eval: det 13/18 DR-0 (roughly matches the prior matched baseline of 10/18), 10/18 DR-0.1 (slightly below the prior baseline's 11/18 -- this member drifted worse post-plateau, consistent with its training-log cert history); sto 0/18 is the known action-noise hold-criterion artifact. Video-verified genuine six-foot recoveries on the failures too (upright, low roll-tail, short-of-height non-falls), no flag/stilt/park exploit. Clearest single demonstration of the cohort's shared NEW finding: RECOVER_GUARD/rollback_count stayed 0 the entire 40M even as this member's own foundational buckets (B0 plant_catch, B1 onefoot_micro, B2 onefoot_mid) collapsed to single digits out of 16 repeatedly in the back half of training, including at the FINAL cert round (7/16, 3/16, 2/16) -- exactly the windowed-pass-rate rollback-trigger gap the any15 dig-in predicted, now confirmed on a run the sync bug does not confound. Once the population stops promoting (frontier plateaus at the tangle wall), members drift independently with no guard catching regressions -- s13 is the clearest cost of that gap. Tangle-family wall (B15 tangle+bank) still unbroken, consistent with the existing CLOSED ruling. Recover/tangle redesign stays operator-gated and outside the SIM SPRINT; no follow-up arm queued.

**refused_reason**: hexapod-mjx-train-3 already runs cw-recover-any21-pop3-s13 — GPU pods host exactly one run; pick a free GPU pod.

