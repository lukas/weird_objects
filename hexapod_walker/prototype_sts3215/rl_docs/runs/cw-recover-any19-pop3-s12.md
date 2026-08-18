# cw-recover-any19-pop3-s12

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INVALID_INTEGRATION_CANARY

**created**: 2026-08-17T23:53:47+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-recover-any18-pop3-s12

**wandb_id**: 1c67c001

**hardware_ready**: False

**hypothesis**: Teach the robot to stand back up from any fallen position by racing three fresh identical-recipe seeds in lockstep and always adopting the first retention-clean winner at each curriculum bucket; this relaunch (member 1, seed 12, of population recover-any19-pop3, roster s11,s12,s13) tests the DETERMINISTIC IDENTITY protocol at 3cc62a2: every member gets a predeclared W&B run id (roster 6907573e,1c67c001,79ef86ae; this member 1c67c001) via wandb.init(id=...,resume=never) and aborts on mismatch, and peers are fetched directly by immutable id with load(force=True) — NO Api.runs/display-name discovery remains, which killed any18: W&B display-name search is eventually consistent and the leader stayed at 1/3 discovery for the full 900s barrier and failed closed. From scratch, absolutely NO --init-from. Never reuse any16/17/18 names. Operator execution directive fb_20260817T234449_bcdcce.

**gate**: Live integration gate (operator fb_20260817T234449_bcdcce): (1) commands contain exact SHA 3cc62a2 or descendant, exact names/seeds/ID roster, no init_from; W&B page IDs exactly 6907573e/1c67c001/79ef86ae, not generated; (2) all 3 stop at exactly 655,360 with valid ready_B00 — no cert/candidate/winner beforehand; (3) once the final run exists, member 0 publishes start_B00; all 3 observe and cross 655,360; (4) first cert on all members reports CERT/recover_training_envs_synchronized=512; (5) exactly one B1 winner; all 3 adopt+ACK identical hash and block; (6) release_B01 only after all 3 identity-bound ACKs — no valid-parent B2 candidate/election before release; (7) all 3 resume from B1 and independently race B2. Fail closed + preserve evidence on any breach. Behavioral claims require genuine video-verified six-foot recovery (no flag/stilt/park exploit).

**verdict**: INVALID_INTEGRATION_CANARY — same cohort story as any19-s11: predeclared W&B ids (3cc62a23) did NOT clear the barrier; this member's log shows 6x 'Could not find run 79ef86ae (not found)' — its DIRECT BY-ID lookups 404'd on s13, the one roster member created AFTER this process started, while controller-side by-id reads of the same run succeeded throughout the window. The process then hung PAST its own 900s barrier deadline (blocked in-process; no RuntimeError, no log flush — the fail-closed timeout is unenforceable when a W&B call blocks). Killed manually at ~00:17Z by the checkup cycle after deadline passage; ready_B00 was valid, no training past 655,360. Root cause (evidence: q_20260817T2352Z addendum): W&B reads from inside a long-lived trainer process are pinned to a stale backend view established at first connect — runs created later stay invisible/404 to that process regardless of fresh wandb.Api() objects, while out-of-process reads always see truth. Fix landed at 8fbb7b21 (fresh GraphQL per read); relaunch is operator-directive-gated.

