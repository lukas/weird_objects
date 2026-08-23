# cw-amp-m2-turnclone-yawcmd-tip50

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T23:56:42+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05-yawcmd-tip50-clockyaw

**wandb_id**: kj5u901l

**hypothesis**: Plain English: if the BC clone is TAUGHT to turn (drove the teacher's own omega channel during data collection, a mechanism that never existed before this cycle) instead of only ever demonstrating straight walking, does the exact same turn-in-place reward/exposure recipe that parked twice (tip50-clockyaw, tip90-clockyaw -- both FAIL-park, tip errs 0.25-0.33 == ~|wz_ref| regardless of the phase-clock fix) finally produce real rotation? Single lever vs tip50-clockyaw: --init-from swapped from the yaw-blind yawcmd checkpoint to a freshly-built ppo_goal_cw_bcgait_turnclone_fullprof_phase1.zip (same teacher, same fullprof/phase1 env contract, +goal.walk_yaw_cmd=1/+drive-omega so TripodGait's native omega channel is demonstrated for the first time -- probe measured the raw teacher achieves 0.15-0.19 rad/s body wz at omega=+/-0.3, and the resulting RAW BC CLONE (zero RL) already scores eval_yaw turn|wz_err| med 0.1035 vs gate 0.1 -- tip errs 0.096/0.108, dramatically better than every RL-trained arm's 0.25-0.40 -- before any fine-tuning). Everything else byte-identical to tip50-clockyaw (frac 0.5, full yaw pricing stack, fresh disc, 2M). Prediction-if-true: eval_yaw tip errs land near/under the already-strong raw-clone baseline (<=0.15) with command-signed wz both directions -- RL polishes an already-present skill instead of inventing one from nothing. Prediction-if-false: RL still washes the skill out (regresses toward the command-invariant park) -- the parking is a property of the SEC5/yawcmd REWARD basin, not the init's prior ignorance, and the next lever is reward-side (mirror-symmetry regularizer or a turn-specific gait-phase term), not a better teacher.

**gate**: Discovery continuation (2M, DR-0). Manual eval_yaw on the run's own pod with the run's own cfg (goal.walk_phase_run_on_yaw=1 included, --speed 0.08 --wz-max 0.3): PASS = tip-left AND tip-right err <=0.20 with achieved wz sign matching wz_ref, zero falls, non-tip translation episodes still gait_valid with prog >=0.7x the yawcmd baseline. PARTIAL = visible improvement over the 0.25-0.33 park fingerprint (say err <=0.20-0.25) without fully clearing the gate -- informative, means RL is polishing the skill, funds a longer/wider follow-up. FAIL-washout = RL regresses back to err ~0.28-0.33 despite the strong raw-clone start -- REWARD-SIDE lever next (mirror-symmetry regularizer), not more init engineering.

