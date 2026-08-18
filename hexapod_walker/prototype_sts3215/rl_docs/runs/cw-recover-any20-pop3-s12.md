# cw-recover-any20-pop3-s12

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: STOPPED_SUPERSEDED

**created**: 2026-08-18T00:30:12+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-recover-any19-pop3-s12

**wandb_id**: 96e80076

**hardware_ready**: False

**hypothesis**: Teach the robot to stand back up from any fallen position by racing three fresh identical-recipe seeds in lockstep and always adopting the first retention-clean winner at each curriculum bucket; this relaunch (member 1, seed 12, of population recover-any20-pop3, roster s11,s12,s13, predeclared W&B ids 7ab95ed5,96e80076,0f713040, this member 96e80076) tests the CACHE-FREE RENDEZVOUS at 8fbb7b2: peer existence and summary reads go through InternalApi.run_resume_status (fresh GraphQL each call) and the Public Api is never constructed during rendezvous/election/ACK/release. From scratch, absolutely NO --init-from. Operator execution directive fb_20260818T001206_0ee733; s11/s12 refused earlier this cycle on stale pod code markers (train-0/1 now synced to 6f50a838).

**gate**: Live integration gate (operator fb_20260818T001206_0ee733): (1) commands at 8fbb7b2 or descendant, exact names/seeds/ID roster, no init_from; (2) all 3 stop at exactly 655,360 with valid ready_B00; (3) leader publishes start_B00 once roster complete, all 3 cross 655,360; (4) first cert on all members CERT/recover_training_envs_synchronized=512; (5) exactly one B1 winner, all 3 ACK identical hash; (6) release_B01 only after all 3 ACKs; (7) all 3 resume from B1 and race B2. Fail closed + preserve evidence on breach.

**verdict**: CORRECTION of the prior INVALID_SPEC note: id 96e80076 is NOT unused -- this row WAS relaunched (respec ~00:46Z with CORRECT any20 roster flags), verified RUNNING with predeclared W&B id 96e80076, crossed the B00 barrier and trained to ~917k steps before being killed ~00:5xZ per fb_20260818T002830 'do not leave a partial cohort running' (s13 stopped 00:39Z, 3-ACK B1 release impossible). CONSEQUENCE: W&B run 96e80076 EXISTS (state finished, step 851968) -- ALL THREE any20 ids are BURNED for resume=never; any21 must mint fresh ids. No behavioral conclusions (bootstrap-era steps). Superseded by cw-recover-any21-pop3.

