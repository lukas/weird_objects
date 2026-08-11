# cw-dep-tall-kh10

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-11T22:09:05+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-dep-tall30

**wandb_id**: ulj0yl2c

**hardware_ready**: False

**hypothesis**: TALL LADDER T2b: k_height crank 10x (100->1000) at ref -15, warm from tall30. At the measured -75mm mid-gait crouch this charges ~5.6/tick, decisively above walk income ~3/tick - the crouch cannot outbid it arithmetically. Risk: freeze/park incentive. T5 probe established the wall is habit, not kinematics: pitch/knee have 45-70deg of upward room.

**gate**: Primary metric: probe_tall_wall.py steady-state walking height (parent = -75mm). PASS: walking height >= -50mm, walk retained (speed >=0.028, survived 1, slip <=1.8, no park). FAIL modes: height flat = pricing refuted at any sane dose; park/freeze = charge too blunt.

**verdict**: INFORMATIVE FAIL (independently confirmed, matches concurrent cycle's T2/T4 verdict logged 22:27). probe_tall_wall.py steady-state walking height -72.6mm (mean over 3 seeds), essentially identical to parent tall30's own -75mm wall and to gate1-h1's -72.6mm -- the 10x k_height crank (100->1000, charging ~5.6/tick at the measured crouch, decisively above walk income) did not move the crouch at all. Leg-yaw still pinned at the 35deg splay limit (margin -0.29 to -0.33deg) in every seed -- same stability-purchase fingerprint. Harness confirms honest six-leg gait (gait_valid 5/6 det+sto, no park/freeze), speed 0.047-0.051 m/s. Pricing family (ref ladder, income gate, gate+budget, height 3x/10x) is CLOSED for posture -- the optimizer cannot FIND the taller basin at any dose, it is not underpaying.

