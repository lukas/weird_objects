# cw-dep-tall-kh10

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-11T22:09:05+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-dep-tall30

**wandb_id**: ulj0yl2c

**hypothesis**: TALL LADDER T2b: k_height crank 10x (100->1000) at ref -15, warm from tall30. At the measured -75mm mid-gait crouch this charges ~5.6/tick, decisively above walk income ~3/tick - the crouch cannot outbid it arithmetically. Risk: freeze/park incentive. T5 probe established the wall is habit, not kinematics: pitch/knee have 45-70deg of upward room.

**gate**: Primary metric: probe_tall_wall.py steady-state walking height (parent = -75mm). PASS: walking height >= -50mm, walk retained (speed >=0.028, survived 1, slip <=1.8, no park). FAIL modes: height flat = pricing refuted at any sane dose; park/freeze = charge too blunt.

