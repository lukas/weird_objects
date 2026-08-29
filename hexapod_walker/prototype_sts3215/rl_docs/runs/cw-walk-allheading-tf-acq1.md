# cw-walk-allheading-tf-acq1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-29T15:35:25+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-walk-allheading-tf-scratch1

**wandb_id**: 35mge3z0

**hypothesis**: Plain English: continue the healthy all-heading transformer walker (2M mechanism canary PASSED: bc-anchor loss falling, course-income mechanism live and recovering through the from-scratch 100Hz reward valley, no NaN/collapse, twin-matched reward) into a real 40M learning budget so it can actually acquire balanced-direction walking. Prediction-if-true: course-income share keeps climbing past the valley (already ticking up in the canary's last 100k steps) and by 40M the balanced 8-heading eval_cmd_suite panel shows every heading moving at >=half the teacher's own completion (>=0.19) with zero falls. Prediction-if-false: course-income share stays pinned near the canary's trough / support keeps falling -- BC-anchor imitation is dominating the objective over command-following and the walker never learns real course-following motion; audit anchor-vs-income coefficient balance before further budget. Strongest alternative: 40M still is not enough to fully exit the valley (08-24 FACT: the MLP reference sibling only crossed zero reward at 12-14M) -- judge interim checkpoints on trend (course-income share direction, bc-anchor loss), never absolute value.

**gate**: 40M acquisition. Cheap first gate (per the canary's own pre-registered gate text): eval_cmd_suite balanced-heading panel, 8 headings x 0.08 m/s + stop, det+sto -- EVERY heading must move (completion >=0.19, half the teacher's measured 0.373-0.385) with zero falls; lateral/reverse weakness = not passed. Full session/mixed-command hardening is a later rung, not this gate.

