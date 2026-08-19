# cw-recover-predictive1b-pop3-s13

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-18T16:54:52+00:00

**pod**: hexapod-mjx-train-9

**steps**: 40000000

**parent**: cw-recover-predictive1-canary1

**wandb_id**: 95414586

**hardware_ready**: False

**hypothesis**: Teach the robot to stand back up from any fallen position, now with a working sense of its own body: three identical fresh seeds race in lockstep, and each policy sees (a) a frozen pretrained dynamics transformer's predictive context in BOTH actor and critic, (b) 15 real sensor frames instead of one repeated frame at the first moment after a fall, and (c) its joint angles measured against the known good stance instead of the arbitrary fallen pose. This cohort tests whether that predictive state context lets from-scratch recovery break the quiet parked-on-four-feet wall (old B14) where the any21 lineage stalled - the new ladder names that failure directly (B14/B15 repair rungs) plus finer tangle steps (B16-B20), bank (B21), flip (B22). Member 2 (seed 13) of population recover-predictive1b-pop3, roster s11,s12,s13, predeclared single-use W&B ids 7901e7bb,304ac843,95414586. FROM SCRATCH: absolutely NO --init-from and NO --recover-init-curriculum; the pretrained transformer is frozen read-only context, never a policy warm start (operator order fb 20260818T161001Z). Same any21 BC mentor/reward/start-bank recipe and synchronized-population/cert/retention/checkpoint protocol; barrier timeout 3600s. Attempt 2 under the operator's retry-once clause; only orchestration parameters (pods, fresh ids, name suffix b) differ from attempt 1.

**gate**: Live integration gate: (1) commands at d37fee09 or descendant; exact seeds 11/12/13; id roster 7901e7bb,304ac843,95414586 exact on every member and W&B pages exactly those ids; (2) from-scratch purity: no --init-from, no --recover-init-curriculum on any member; encoder md5 9df48f687967c25085ee50171e4110ff verified at load on all 3; (3) all 3 stop at the bootstrap barrier (655,360) with valid ready_B00, leader releases start_B00, all 3 cross; (4) first cert on all members reports CERT/recover_training_envs_synchronized=512; (5) every promotion follows the unchanged election protocol (one winner, identical-hash adoption+ACK on all 3 before release); (6) fail closed + preserve evidence on any breach; never silently continue a partial cohort. Behavioral verdict at the pre-registered 40M checkpoints on the 23-rung ladder: video-verified genuine six-foot recovery (no flag/stilt/park exploit), frontier judged against the any21 cohort's B14 wall.

**verdict**: Cohort member (seed 13) completes the full 40M budget and is the first checkpoint in the recover lineage to clear the ENTIRE 23-rung ladder including the explicit B14/B15 repair rungs built to teach the old parked-on-4/5-feet wall: training frontier reached B22 (flip, the hardest rung) by 37.7M steps and held there through 40M, CERT/recover_training_envs_synchronized=512 at all 40 cert rounds (population sync clean the whole run). One RECOVER_GUARD rollback correctly fired mid-run (trigger bucket 11 at ~30M, restored to B19, then re-progressed past it to B22 -- the retention guard worked as designed this time). Matched one-shot gate eval (n=23, new ladder): DR-0 det 21/23 recover_success (only real miss = flip, video shows a stalled/awkward first-ever attempt, not an exploit; the other miss, bucket 11 zero, is video-confirmed a CLEAN quiet stand -- same known quiet-hold-timing false-negative documented on the any21 line, not a real failure). Own-DR-0.1 det 23/23 (incl. flip, video-verified a genuine if imperfect recovery). STO 0/23 both passes -- reviewed sto video (buckets 13,20) shows the same known artifact: clean, level, six-foot stands held the whole clip, action noise just keeps missing the strict quiet-hold termination window, not a new defect. No flag-leg/tripod/stilt/park exploit in any reviewed video. This is genuine, video-verified progress past the operator-named old B14 wall on ONE cohort member; s11/s12 (same lineage, still training) get their own verdicts, and only once all three are in does this become a cohort-level claim. Recover/tangle redesign stays operator-gated per the standing ruling -- no follow-up arm queued from this cycle regardless of the result.

