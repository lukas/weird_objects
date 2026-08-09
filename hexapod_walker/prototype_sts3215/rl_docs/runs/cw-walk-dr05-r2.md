# cw-walk-dr05-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-09T11:22:47+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-walk-anchorgate

**hypothesis**: OPERATOR LAUNCH retry (r1 crashed on missing init-from checkpoint, now fixed). HIGHER DR: train champion 35234ddc at dr-scale 0.5 instead of no-dr; robustness vs brittle-tuning. If-true: DR0.5 det 6/6 gait_valid zero-term slip <=1.24 + DR0 retention.

**gate**: DR0.5 det+sto 6/6: zero terminations, gait_valid, det slip/m <= 1.24; DR0 det retention 6/6

**refused_reason**: hexapod-mjx-train-2 code marker ecf9dc320288c0b642e98feecb322891de4fa57d-dirty != local HEAD ecf9dc320288c0b642e98feecb322891de4fa57d. Sync first: snapshot.sh --sync hexapod-mjx-train-2 (and snapshot/commit before that if the tree is dirty).

