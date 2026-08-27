# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2-headings-curric1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-27T06:28:40+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2-headings1

**wandb_id**: lcde9n5h

**hypothesis**: Plain sentence: does RAMPING the heading command cone open gradually over training (instead of opening it fully at step 0, which just-verdicted headings1/-s1 showed produces no reliable direction-learning signal) let the walk core actually learn to steer? Single mechanism change vs anchor2-headings1: goal.walk_heading_max_rad is no longer a static 0.7854 from tick 0 -- it now ramps 0.0 -> 0.7854 linearly over the first 1.2M of this 2M continuation via the existing sched.* engine (sched.key=goal.walk_heading_max_rad, same generic tool the joystick track's steer2-stagecurric1 arm already validated for exactly this class of failure: 'full exposure from step 0 is not enough, ramp difficulty instead'). Same anchor2 leak-fixed base, same goal-mix, same everything else. Prediction-if-true: direction_err_mean_deg on a heading-opened walk eval drops well below both the ~52deg untrained baseline AND headings1's own best read (~36deg DR-0 det only) -- ideally <=35deg full bar, robust across DR-0 and own-DR, not just one slice -- while walk still survives (det gait_valid >=5/6). Prediction-if-false: direction error stays at baseline regardless of ramp pace, meaning the failure is not an exposure-schedule problem (points at the course-reward pricing at nonzero headings, or the walk core's obs/action authority for turning, dig-in scope).

**gate**: MECHANISM-HEALTH CANARY (2M) + DIRECTION READ, joint 2-seed call with -s1. WALK-SURVIVES: det gait_valid >=5/6 both DR, no anchor1/anchor4-class leg-sacrifice freeze, prog_ratio >=0.2. DIRECTION-LEARNS-FULL: median direction_err_mean_deg <=35deg on BOTH DR-0 det AND own-DR(0.5) det (not just one slice -- headings1's own DR-0-only partial that washed out under DR is the bar this must clear to promote). PARTIAL: DR-0 det clears <=35 or shows a bigger drop than headings1's ~36deg median but own-DR still doesn't hold -- promote to a longer/full-ramp-length continuation. FAIL: direction error at either DR stays within noise of headings1's own numbers (no improvement from ramping) -- closes the exposure-schedule theory, points dig-in at course-reward pricing or turning authority instead.

