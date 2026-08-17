# cw-recover-any17-pop3-s11

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-17T22:58:26+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-recover-any16-pop3-s11

**wandb_id**: 5zvb4x7p

**hypothesis**: Teach the robot to stand back up from any fallen position by racing three fresh identical-recipe seeds in lockstep and always adopting the first retention-clean winner at each curriculum bucket; this relaunch (member 0, seed 11, of population recover-any17-pop3, roster s11,s12,s13) tests the CORRECTED synchronization protocol at f5aee3f: every member trains exactly 10 rollouts (655,360 steps), publishes ready_B00 and BLOCKS until member 0 releases start_B00; each bucket's first retention-clean winner is hash-verified, adopted (policy+optimizer+curriculum) and ACKed by all three before the leader publishes release_BNN and anyone continues — fixing the stale _peer_rows cache + missing release barrier that let any16 member 0 run ahead on its own private lineage. From scratch, absolutely NO --init-from. Supersedes the INVALID any16 cohort (never resume it).

**gate**: Live integration gate (operator fb_20260817T225114_a31958): (1) all 3 cohort rows RUNNING, distinct W&B IDs, seeds 11/12/13, code SHA f5aee3f or descendant, NO init_from; (2) each trains exactly 10 rollouts to 655,360 steps and logs WAITING at the initial B0 bootstrap barrier — no cert/candidate/winner before start_B00; (3) member 0 publishes start_B00 only after all three ready_B00 records, all three log start observed and resume; (4) first cert on all three reports CERT/recover_training_envs_synchronized=512; (5) at first promotion exactly one B1 winner is elected and all three adopt/ACK the same policy hash and BLOCK; (6) leader publishes release_B01 only after all three identity-bound ACKs — no valid-parent B2 candidate/election before release_B01; (7) after release_B01 all three resume from the exact B1 winner checkpoint with distinct seed/RNG streams and race the next bucket. Fail closed + preserve evidence on any breach; never silently continue a partial cohort. Behavioral: frontier claims require video-verified genuine six-foot recoveries, no flag/stilt/park exploit.

