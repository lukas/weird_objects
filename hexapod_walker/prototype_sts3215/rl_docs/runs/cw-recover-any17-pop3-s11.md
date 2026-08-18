# cw-recover-any17-pop3-s11

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: STALE_DUPLICATE

**created**: 2026-08-17T23:01:29+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**hypothesis**: Teach the robot to stand up from any fallen position by racing three fresh training seeds and always continuing from the best one; the three seeds (11/12/13) train in lockstep bucket races: every seed trains exactly 10 rollouts (655,360 steps) then waits at a start barrier; after each retention-clean promotion the winning seed's exact checkpoint is adopted, hash-verified and ACKed by all three, and only then does the leader release the next race — so every curriculum bucket starts three branches from the identical checkpoint. This replaces the any16-pop3 canary, which the operator stopped after live audit showed cached W&B summaries and a missing post-ACK release barrier let member 0 run ahead unsynchronized; both defects are fixed at f5aee3f. From scratch, NO init-from; population recover-any17-pop3, roster s11,s12,s13. Member 0, seed 11.

**gate**: Live integration gate (operator, fb_20260817T225114_a31958): (1) 3 ledger rows RUNNING, distinct W&B IDs, seeds 11/12/13, code SHA containing f5aee3f, no init_from; (2) each trains exactly 655,360 steps (10 rollouts) then logs WAITING at the initial B0 bootstrap barrier — no cert/candidate/winner before start_B00; (3) member 0 publishes start_B00 only after all three ready records, all three log start observed and resume; (4) first cert reports CERT/recover_training_envs_synchronized=512 on all three; (5) at first promotion exactly one B1 winner elected, all three adopt/ACK the same policy hash and BLOCK; (6) leader publishes release_B01 only after all three ACKs — no valid-parent B2 candidate/election before release_B01; (7) after release_B01 all three resume from the B1 checkpoint and race the next bucket. Fail closed + preserve evidence on any gate failure; no partial cohort continues. Behavioral frontier claims require genuine six-foot recoveries (no flag/stilt/park).

**verdict**: Stale duplicate ledger row from a launcher refusal — NOT the scientific verdict for s11. An earlier s11 process really did run (W&B 5zvb4x7p, the 2026-08-17T22:58:26Z row), which is verdicted INVALID_INTEGRATION_CANARY there. Reconciled per operator fb_20260817T231211_ba01c4.

**refused_reason**: hexapod-mjx-train-0 already runs cw-recover-any17-pop3-s11 — GPU pods host exactly one run; pick a free GPU pod.

