# cw-dep-fresh1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T04:33:37+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**wandb_id**: qmtctc9g

**hypothesis**: OPERATOR PRIORITY (hardware session 08-09 night, RL_LOG session 3). Fresh-init deployment-contract arm, the rocking-gait question: the scripted tripod gait that ACTUALLY WALKS the robot rocks +-10-20deg roll/pitch, which our 10deg tilt termination forbids -- our RL line may be locked into low-amplitude creep because real weight transfer is a termination event. Hypothesis: from scratch under the deployed contract (meas:=ref velocity obs via goal.walk_obs_body_vel=2, 25deg tilt envelope, field-standard exploration log_std 0 / ent 0.005 per BEST_PRACTICES_AUDIT) a weight-transfer gait emerges that walks with body rock like the scripted gait instead of creeping. If FALSE (converges to the same creep or fails gait_valid), rocking-permission alone is not the creep bottleneck.

**gate**: own-cfg det+sto 6/6 @15s gait_valid 12/12, 0 term; video frames watched SPECIFICALLY for weight transfer vs creep + roll amplitude report vs the +-10-20deg hardware envelope; slip/m reported (feet MAY slide per hardware finding f)

