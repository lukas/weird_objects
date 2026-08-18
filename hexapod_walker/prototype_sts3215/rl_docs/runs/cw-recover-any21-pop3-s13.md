# cw-recover-any21-pop3-s13

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-18T00:49:46+00:00

**pod**: hexapod-mjx-train-3

**steps**: 40000000

**parent**: cw-recover-any19-pop3-s13

**wandb_id**: fe8501ac

**hypothesis**: Teach the robot to stand back up from any fallen position by racing three fresh identical-recipe seeds in lockstep and always adopting the first retention-clean winner at each curriculum bucket; this cohort (member 2, seed 13, of population recover-any21-pop3, roster s11,s12,s13, predeclared W&B ids f14d9993,a705c488,fe8501ac, this member fe8501ac) completes the any21 roster after s11 (RUNNING, waiting at its B0 barrier) — any20 was lost to a pure orchestration race (member 2 stopped 20s before a successful s11/s12 repair), not a code defect; this is the operator-preauthorized fallback fb_20260818T002830_3d14e2, re-synced to current HEAD after a code-marker race on the first s12 attempt. From scratch, absolutely NO --init-from. Never reuse any16-any20 names.

**gate**: Live integration gate (inherited from operator fb_20260818T001206_0ee733, roster per fallback fb_20260818T002830_3d14e2): (1) commands at 8fbb7b2 or descendant, exact names/seeds, id roster f14d9993/a705c488/fe8501ac exact on every member, no init_from; (2) all 3 stop at exactly 655,360 with valid ready_B00; (3) leader publishes start_B00 once roster complete, all 3 cross 655,360; (4) first cert on all members CERT/recover_training_envs_synchronized=512; (5) exactly one B1 winner, all 3 ACK identical hash; (6) release_B01 only after all 3 ACKs; (7) all 3 resume from B1 and race B2. Fail closed + preserve evidence; never silently continue a partial cohort.

**refused_reason**: hexapod-mjx-train-3 already runs cw-recover-any21-pop3-s13 — GPU pods host exactly one run; pick a free GPU pod.

