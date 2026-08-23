# cw-amp-m2-turnclone-yawcmd-tip50-yawprice3x

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-23T00:24:50+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-amp-m2-turnclone-yawcmd-tip50

**hypothesis**: Plain English: tip50's own verdict found 2M of RL under the unchanged yaw pricing recipe made turning WORSE than the raw untrained BC-turn-clone (eval_yaw err 0.1035 raw -> 0.140 after RL), while training reward rose the whole time -- an 08-21-style misalignment (reward up, target metric down), because yaw income (k_walk_yaw=1.0, k_yaw_prog=1.0) is priced too low relative to translation+style income to protect the newly-available turning skill during fine-tuning. Single lever vs tip50 (continuing from ITS OWN checkpoint, not restarting): triple the two yaw income terms (k_walk_yaw 1.0->3.0, k_yaw_prog 1.0->3.0), everything else byte-identical (same 0.5 turn-in-place exposure, same style weight, same fresh-disc-already-warm state).

**gate**: Discovery continuation (2M, DR-0). eval_yaw on the run's own pod cfg (--speed 0.08 --wz-max 0.3, goal.walk_phase_run_on_yaw=1 included): PASS if turn err <=0.10 (closes the M2-yaw gate outright). INFORMATIVE/repriced-helps if turn err improves toward/below the raw clone's own 0.1035 without regressing translation gait_valid below 10/12. NO-CHANGE/repriced-doesn't-help if err stays ~0.14 or worse despite 3x pricing -- then the bottleneck is not income magnitude (structural lever, e.g. mirror-symmetry regularizer, is next).

