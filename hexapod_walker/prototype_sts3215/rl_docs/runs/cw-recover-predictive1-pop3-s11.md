# cw-recover-predictive1-pop3-s11

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-18T16:42:13+00:00

**pod**: hexapod-mjx-train-5

**steps**: 40000000

**parent**: cw-recover-predictive1-canary1

**wandb_id**: 20827091

**hypothesis**: Teach the robot to stand back up from any fallen position, now with a working sense of its own body: three identical fresh seeds race in lockstep, and each policy sees (a) a frozen pretrained dynamics transformer's predictive context in BOTH actor and critic, (b) 15 real sensor frames instead of one repeated frame at the first moment after a fall, and (c) its joint angles measured against the known good stance instead of the arbitrary fallen pose. This cohort tests whether that predictive state context lets from-scratch recovery break the quiet parked-on-four-feet wall (old B14) where the any21 lineage stalled - the new ladder names that failure directly (B14/B15 repair rungs) plus finer tangle steps (B16-B20), bank (B21), flip (B22). Member 0 (seed 11) of population recover-predictive1-pop3, roster s11,s12,s13, predeclared single-use W&B ids 20827091,2dc580a7,200e6aac. FROM SCRATCH: absolutely NO --init-from and NO --recover-init-curriculum; the pretrained transformer is frozen read-only context, never a policy warm start (operator order fb 20260818T161001Z). Same any21 BC mentor/reward/start-bank recipe and synchronized-population/cert/retention/checkpoint protocol; barrier timeout 3600s (orchestration robustness only, learned from the any21c skew loss).

**gate**: Live integration gate: (1) commands at d37fee09 or descendant; exact names/seeds 11/12/13; id roster 20827091,2dc580a7,200e6aac exact on every member and W&B pages exactly those ids; (2) from-scratch purity: no --init-from, no --recover-init-curriculum on any member; encoder md5 9df48f687967c25085ee50171e4110ff verified at load on all 3; (3) all 3 stop at the bootstrap barrier (655,360) with valid ready_B00, leader releases start_B00, all 3 cross; (4) first cert on all members reports CERT/recover_training_envs_synchronized=512; (5) every promotion follows the unchanged election protocol (one winner, identical-hash adoption+ACK on all 3 before release); (6) fail closed + preserve evidence on any breach; never silently continue a partial cohort. Behavioral verdict at the pre-registered 40M checkpoints on the 23-rung ladder: video-verified genuine six-foot recovery (no flag/stilt/park exploit), frontier judged against the any21 cohort's B14 wall.

**verdict**: Stopped deliberately at the bootstrap barrier (655,360 steps, pre-divergence): member 2 (s13) crashed at boot on train-8 missing v5_mjx_fresh dataset and burned its single-use predeclared id 200e6aac, so the baked-in roster can never rendezvous. Fail closed, never continue a partial cohort. Healthy while it ran (encoder verified, Warp live). Re-run: cw-recover-predictive1b-pop3 with fresh ids 7901e7bb,304ac843,95414586 on dataset-equipped pods 5/7/9.

