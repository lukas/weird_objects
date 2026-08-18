# cw-recover-any21-pop3-s12

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS (partial)

**created**: 2026-08-18T00:49:23+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-recover-any19-pop3-s12

**wandb_id**: a705c488

**hardware_ready**: False

**hypothesis**: Teach the robot to stand back up from any fallen position by racing three fresh identical-recipe seeds in lockstep and always adopting the first retention-clean winner at each curriculum bucket; this cohort (member 1, seed 12, of population recover-any21-pop3, roster s11,s12,s13, predeclared W&B ids f14d9993,a705c488,fe8501ac, this member a705c488) completes the any21 roster after s11 (RUNNING, waiting at its B0 barrier) — any20 was lost to a pure orchestration race (member 2 stopped 20s before a successful s11/s12 repair), not a code defect; this is the operator-preauthorized fallback fb_20260818T002830_3d14e2, re-synced to current HEAD after a code-marker race on the first s12 attempt. From scratch, absolutely NO --init-from. Never reuse any16-any20 names.

**gate**: Live integration gate (inherited from operator fb_20260818T001206_0ee733, roster per fallback fb_20260818T002830_3d14e2): (1) commands at 8fbb7b2 or descendant, exact names/seeds, id roster f14d9993/a705c488/fe8501ac exact on every member, no init_from; (2) all 3 stop at exactly 655,360 with valid ready_B00; (3) leader publishes start_B00 once roster complete, all 3 cross 655,360; (4) first cert on all members CERT/recover_training_envs_synchronized=512; (5) exactly one B1 winner, all 3 ACK identical hash; (6) release_B01 only after all 3 ACKs; (7) all 3 resume from B1 and race B2. Fail closed + preserve evidence; never silently continue a partial cohort.

**verdict**: Full 40M budget complete (integration gate already proven live this cycle-window). Matched gate-eval (n=18, same convention as the any11/any15 reference): DET 16/18 at DR-0 and 14/18 at DR-0.1 — a genuine new best for the recover line (prior reference 10/18 and 11/18) and reaches a materially higher curriculum frontier (tangle_deep/tangle+bank, bucket 14/15, vs the old B8 partial_high wall); only real det miss is tangle_mid (over_current, a genuine torque-limit fumble, not an exploit) plus the never-trained flip bucket. STO is 0/18 at both DR levels, unchanged from the established baseline — but reviewed sto videos (plant_catch, partial_high) show clean, level, six-foot stands with no fall/tangle/exploit; the failures are timeouts against the strict quiet-hold criterion (report.json: no TERM at all, not a crash), matching REWARD.mds documented stochastic strict-hold false-negative, not a new defect. Visual-quality caveat: even successful DET recoveries carry a real residual lean (roll tail ~24 deg, roll_settled 0/18 by the <2 deg bar) — not clean/level, a genuine hardware-readiness gap. Training-side: this members own curriculum reached frontier B14 fast (~15M steps) then STALLED for the remaining ~25M steps (62% of budget) with retention oscillating (bucket 0/plant_catch itself repeatedly crashed to 0-44% pass mid-run) before recovering by the final round -- the SAME stability-wall pattern already diagnosed at any11/any15s B7/B8, now recurring one rung higher at the tangle boundary; the sync/broadcast fix proved the MECHANISM, not the underlying stability wall. No flag/stilt/park exploit in any reviewed video. Verdict scoped to s12 only (its own post-B14 training diverged from s11/s13); not hardware-ready; recover/tangle redesign stays operator-gated per CURRENT_TRUTHS -- no autonomous follow-up queued.

**refused_reason**: a process for cw-recover-any21-pop3-s12 already exists on hexapod-mjx-train-1

**note**: Ledger extra_args are stale for this row: the recorded vector carries --recover-population-member=0 from a pre-correction attempt, but the LIVE process (verified via W&B) initialized as member 1 with predeclared id a705c488, seed 12, roster f14d9993,a705c488,fe8501ac — correct per fb_20260818T002830 direct-relaunch prescription. Do not respec from this row without overriding all population flags.

