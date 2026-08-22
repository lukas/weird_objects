# cw-amp-m2-bcinit-sec5-style05-yawcmd-tip50-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T22:42:02+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05-yawcmd

**wandb_id**: 266dask7

**hypothesis**: Plain English: the AMP walker ignores turn commands because it almost never SEES them -- dedicated turn-in-place states are only ~7.5% of segments under independent sampling; does making HALF of all episodes whole-episode turns (goal.walk_turn_in_place_frac=0.5: zero linear command, guaranteed non-trivial wz with a 50/50 sign draw) finally teach command-signed turning, while the other half preserves translation? Continues from the yawcmd checkpoint (74-dim obs with the yaw channel + the full bank-verified yaw pricing stack already in place; teacher_v2 has REAL turn-in-place clips so the style channel rewards genuine turning). This is the operator-directed 08-10 lever (command EXPOSURE, not more price tuning -- see walk_task.py tip_frac comment) never yet tried on the AMP substrate; yawcmd just proved pricing alone is outbid (yaw_err rose while reward rose; robot parks on tip commands, err == |wz_ref|). Prediction-if-true: eval_yaw tip-left/right err drops well below the 0.30 park fingerprint with achieved wz SIGN matching wz_ref in both directions, gait_valid >=5/6 det+sto, translation dir_err within 15deg of yawcmd (det 32.9/sto 49.0). Prediction-if-false: robot still parks on tip episodes (err ~0.30) despite 50% exposure -- turning is blocked below the command level (phase-locked tripod / style conflict); next lever is a mirror-symmetry regularizer or turn-specific gait phase, not more exposure. Strongest alternative: it turns but only in one direction (chirality), or translation erodes >15deg (curriculum ordering problem). (-r2: attempts 1-2 refused on pod code-marker races with a concurrent cycle's snapshots; nothing trained under earlier names.)

**gate**: Discovery (2M, DR-0). PASS = eval_yaw (run manually on the pod with the run's own cfg: --speed 0.08 --wz-max 0.3 + the goal.* cfg-set list) shows command-SIGNED turn-in-place: tip-left AND tip-right err <= 0.20 (vs yawcmd's 0.2995/0.3008 park fingerprint), achieved wz sign matching wz_ref for BOTH signs, zero falls, AND the standard DR-0 gate holds gait_valid >=5/6 det+sto with translation dir_err med within 15deg of yawcmd (det 32.9/sto 49.0). FAIL-park = tip errs still ~0.30 (exposure refuted at 0.5 dose -- read jointly with the tip90 twin). FAIL-chiral = only one sign turns (structural asymmetry finding). FAIL-erode = turning emerges but translation degrades >15deg (stage-ordering problem, informative not dead-end).

