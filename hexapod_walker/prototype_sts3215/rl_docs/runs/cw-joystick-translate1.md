# cw-joystick-translate1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-15T12:10:14+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-mt-c2

**wandb_id**: ti7hygbp

**hypothesis**: Teach one policy to walk wherever the joystick points: continue cw-mt-c2 - which was still actively improving when its 20M budget ended but had learned to bank income by dragging on five legs and falling early - with joystick translation commands covering every direction (heading uniform [-pi,pi], speed 0.03-0.06 m/s, no stop segments), 60 s sessions, the all-support-leg gait income gate, and a fall cost that grows with how early the robot dies, to test whether c2's collapse was an unsafe-reward mismatch rather than an inability to learn broad commands (operator directive fb_20260815T114414_3c40d6). Translation only: wz identically zero (yaw obs channel kept for checkpoint width), no turning claim - the fb_20260815T113718_baf9d6 audit showed c2 was never paid for yaw. Headline: joystick/v_along_m_s (+_cumulative) = average signed m/s in the requested joystick direction over nonzero-command ticks. Prediction-if-true: it climbs while falls shrink. Prediction-if-false: it plateaus near zero or survival never improves while task reward stalls - broad-command acquisition genuinely fails at this capacity.

**gate**: Held-out EVAL only for direction splits: eval_cmd_suite 12-direction panel (headings every 30deg, 0.045 m/s, wz=0) + randomized 60 s command sessions, det+sto: zero falls; no sacrificed leg (per-leg duty_cycle >= 0.10 on all six); positive raw v_along in EVERY panel direction with det med v_along >= 0.015 m/s (>= 1/3 of mean command; aggregate reward alone never passes); v_cross_abs med <= 0.03 m/s. No automatic FAIL verdict while joystick/v_along_m_s + env/reward_task are still materially rising (operator: stop early only on numerical/mechanical failure or pre-registered safety regression). Verdict-bearing PASS additionally requires the bulk_session_eval cohort per EVALS.md sec 4.

