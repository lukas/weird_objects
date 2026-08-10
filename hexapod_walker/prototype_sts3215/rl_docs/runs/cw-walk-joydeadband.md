# cw-walk-joydeadband

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-09T23:41:16+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-joylat25

**hypothesis**: Compose rung off joylat25 (driving package): friction/tilt/payload have all been composed onto the widest/latency-hardened driving lines but servo deadband (proven trainable at DR0/DR0.5 on the walk-only line, deadband30/deadband_dr05) has not yet been tried WHILE steering. One variable off joylat25: add dr.deadband_scale=1.0,3.0. If-true: JOYSTICK GATE 0 falls AND own-cfg (DR0.5+latency+deadband) gv 12/12, 0 term, prog med >=0.80 -- deadband joins the driving-hardened axis set. If-false: deadband draws break flip recovery or crater progress under steering (unlike the walk-only compose) -- steering is deadband-fragile even though straight walking wasn't. Strongest alternative: gate passes but slip/prog shades vs joylat25 baseline -- compare per-episode.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 --heading-max-deg 45 -- ZERO in-envelope falls; own-cfg (DR0.5+latency0.5-2.5x+deadband1-3x) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog_ratio med >=0.80; DR0 retention det 6/6 gv; frames watched det

**failed_reason**: run never appeared as 'running' in W&B within 240s

