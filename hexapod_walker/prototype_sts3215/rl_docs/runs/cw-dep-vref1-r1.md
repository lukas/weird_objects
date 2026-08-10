# cw-dep-vref1-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T04:18:47+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-lowgait-dr05-r1

**hypothesis**: OPERATOR PRIORITY (hardware session 08-09 night, RL_LOG session 3). Deployment-contract arm: hardware feeds walk obs vx/vy_meas:=ref (board has no velocity estimate; NEW goal.walk_obs_body_vel=2 matches it exactly) and a WORKING gait rocks +-10-20deg roll/pitch (measured, scripted gait) vs our 10deg tilt termination. Hypothesis: champion warm-start under the exact deployed contract (meas:=ref + 25deg tilt envelope) preserves gait quality; if FALSE (gait_valid or slip degrades >20% vs parent) the champion depends on privileged velocity -> escalate temporal-actor/estimator to P0. (r1: first attempt cw-dep-vref1 died 0-step to the launch-collision storm + shm SIGBUS on train-7; wandb 33seo7fp is the corpse, no science.)

**gate**: own-cfg det+sto 6/6 @15s gait_valid 12/12, 0 term, vel err within 15% of parent on same eval; video frames watched; explicit parent comparison

