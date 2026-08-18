# cw-recover-any20-pop3-s11

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INVALID_INTEGRATION_CANARY

**created**: 2026-08-18T00:27:59+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-recover-any19-pop3-s11

**wandb_id**: 7ab95ed5

**hardware_ready**: False

**hypothesis**: Teach the robot to stand back up from any fallen position by racing three fresh identical-recipe seeds in lockstep and always adopting the first retention-clean winner at each curriculum bucket; this relaunch (member 0, seed 11, of population recover-any20-pop3, roster s11,s12,s13, predeclared W&B ids 7ab95ed5,96e80076,0f713040, this member 7ab95ed5) tests the CACHE-FREE RENDEZVOUS at 8fbb7b2: peer existence and summary reads go through InternalApi.run_resume_status (fresh GraphQL each call) and the Public Api is never constructed during rendezvous/election/ACK/release. From scratch, absolutely NO --init-from. Operator execution directive fb_20260818T001206_0ee733; s11/s12 refused earlier this cycle on stale pod code markers (train-0/1 now synced to 6f50a838).

**gate**: Live integration gate (operator fb_20260818T001206_0ee733): (1) commands at 8fbb7b2 or descendant, exact names/seeds/ID roster, no init_from; (2) all 3 stop at exactly 655,360 with valid ready_B00; (3) leader publishes start_B00 once roster complete, all 3 cross 655,360; (4) first cert on all members CERT/recover_training_envs_synchronized=512; (5) exactly one B1 winner, all 3 ACK identical hash; (6) release_B01 only after all 3 ACKs; (7) all 3 resume from B1 and race B2. Fail closed + preserve evidence on breach.

**verdict**: CORRECTED (W&B ground truth): the run DID train — after the malformed retries died at argparse, a corrected direct relaunch (~00:36Z) took predeclared id 7ab95ed5, reached the 655,360 barrier, and as leader RELEASED start_B00 at ~00:39Z (wait 0.34s) — the FIRST successful live rendezvous of the 8fbb7b2 InternalApi protocol; it crossed to ~983k steps before being stopped ~00:41Z as an unrecoverable partial cohort (s13 had been stopped 20s before the release by the fb_20260818T002830 fallback racing the repair; predeclared ids are single-use so the cohort could not be completed). Integration evidence only, no behavioral claims; superseded by recover-any21-pop3 which passed gates 1-4 live.

