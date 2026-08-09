# cw-walk-fwdband-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-09T12:37:14+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_anchorgate.zip

**hypothesis**: RETRY of cw-walk-fwdband (r1 launch died at INTENT when its cycle was killed; 0 steps, infra fault). The robot is only ever graded on walking straight ahead (operator ruling 4), yet 40% of practice commands point sideways/backward. 100% forward scope off champion 35234ddc: if focused practice matters, det prog_ratio moves into 0.75-1.25 (champ 1.43 overspeed) without slip worsening; if nothing moves beyond noise, command mix is not a lever. A/B partner: cw-steer-fdiag (pi/4).

**gate**: own-cfg DR1.0 harness 15s 6+6 fwd: det prog_ratio median 0.75-1.25 (champ 1.43), det slip_per_m <=1.11 (not worse beyond noise), gait_valid 12/12, 0 term; DR0 det fwd >=0.55 6/6; frames watched det+sto

