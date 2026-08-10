# cw-dep-vref1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T03:58:35+00:00

**pod**: hexapod-mjx-train-7

**steps**: 20000000

**parent**: cw-walk-lowgait-dr05-r1

**hypothesis**: Deployment-contract arm (operator hardware session 08-09 night, RL_LOG session 3): hardware feeds walk obs vx/vy_meas:=ref (board has no velocity estimate) and a WORKING gait rocks +-10-20deg roll/pitch (scripted-gait measurement) while our line trains with privileged sim velocity and a 10deg tilt termination. Warm-starting the current walk champion with goal.walk_obs_body_vel=2 (meas:=ref, exact deployed contract) + 25deg tilt envelope preserves gait quality (no erosion) and produces a checkpoint that is contract-identical to deployment. If FALSE (gait degrades >20% on gait_valid or slip), the champion secretly depends on privileged velocity -> escalate temporal-actor/estimator line to P0.

**gate**: own-cfg det+sto 6/6 @15s gait_valid 12/12, 0 term, vel err within 15% of parent baseline; deployment-pipeline eval (meas:=ref at gate time is now IDENTICAL to train time - the point); video frames watched; explicit comparison vs parent on same eval

**failed_reason**: run never appeared as 'running' in W&B within 240s

