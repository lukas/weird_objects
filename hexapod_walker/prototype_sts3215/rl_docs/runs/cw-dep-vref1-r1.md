# cw-dep-vref1-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T04:18:47+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-lowgait-dr05-r1

**wandb_id**: bhvjen1w

**hardware_ready**: False

**hypothesis**: OPERATOR PRIORITY (hardware session 08-09 night, RL_LOG session 3). Deployment-contract arm: hardware feeds walk obs vx/vy_meas:=ref (board has no velocity estimate; NEW goal.walk_obs_body_vel=2 matches it exactly) and a WORKING gait rocks +-10-20deg roll/pitch (measured, scripted gait) vs our 10deg tilt termination. Hypothesis: champion warm-start under the exact deployed contract (meas:=ref + 25deg tilt envelope) preserves gait quality; if FALSE (gait_valid or slip degrades >20% vs parent) the champion depends on privileged velocity -> escalate temporal-actor/estimator to P0. (r1: first attempt cw-dep-vref1 died 0-step to the launch-collision storm + shm SIGBUS on train-7; wandb 33seo7fp is the corpse, no science.)

**gate**: own-cfg det+sto 6/6 @15s gait_valid 12/12, 0 term, vel err within 15% of parent on same eval; video frames watched; explicit parent comparison

**verdict**: PASS -- contract-exact obs (meas:=ref for vx/vy) + 25deg tilt envelope does NOT erode the champion gait; if-false (escalate estimator/temporal-actor to P0) is REFUTED. Own-cfg harness det+sto 6/6 gv, 0 term. Named baseline: parent lowgait_dr05_r1 evaluated on the identical eval config (same DR-scale 0.0, seed 0, episode-seconds 15, same reward cfg, minus the contract change) gives det slip/m med 0.97 vel_err 0.025 fwd 0.75m, sto slip/m med 1.36 vel_err 0.031 fwd 0.68m. Child (contract-exact) gives det slip/m med 0.89 vel_err 0.024 fwd 0.78m, sto slip/m med 1.13 vel_err 0.031 fwd 0.69m -- vel_err delta 0/4% (both modes), slip/m actually BETTER not worse, fwd distance matches within noise. Both have one isolated degraded sto episode (parent ep4 prog 0.34 slip 4.35; child ep4 prog 0.26 slip 5.97) -- same lineage fixed-draw-stall pattern at the same draw index, not a new pathology. Video (det + sto frame strips, all 6 det + sto_4) shows the same low-amplitude six-leg creep gait as the parent, no flag leg, no visible erosion. RULING: contract-exact obs is SUFFICIENT for hardware attempt #2 -- velocity estimator/temporal actor is NOT a P0 prerequisite (GPT handoff item 2). Not hardware-ready itself (still paddle-gait, high slip vs the real floor); this checkpoint's role is to unblock cw-dep-startvar1 (start-variation compose), now correctly parented.

