# cw-walk-dr05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-09T11:08:01+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-walk-anchorgate

**hypothesis**: OPERATOR LAUNCH (parallel-lines directive): HIGHER DR. The champion lineage trains at DR 0 and is only EVALED at DR 1.0; training under domain randomization 0.5 (mass/geometry/friction/compliance/gravity/gains per MJX model-field DR) tests whether the gait is robust or brittle-tuned. One config change off champion ppo_goal_cw_walk_anchorgate.zip md5 35234ddc: --no-dr -> --dr-scale 0.5. If-true: DR0.5 det 6/6 gait_valid zero-term with slip/m <= 1.24, and DR0 retention holds. If-false (gait collapses under DR): the paddle is DR-fragile and robustness must precede more income shaping. Snapshot 98c7fb7.

**gate**: DR0.5 det+sto 6/6: zero terminations, gait_valid, det slip/m <= 1.24; DR0 det retention 6/6

**verdict**: DEAD at init (0 steps): FileNotFoundError on parent ckpt ppo_goal_cw_walk_anchorgate.zip on mjx-train-2 (snapshot.sh --sync excludes policies/). W&B tqb1qjaf state=failed. Retried once as cw-walk-dr05-r1 in cycle 35 after pushckpt + md5 verify.

**failed_reason**: run never appeared as 'running' in W&B within 240s

