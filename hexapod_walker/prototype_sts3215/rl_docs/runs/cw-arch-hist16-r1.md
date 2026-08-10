# cw-arch-hist16-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T00:47:26+00:00

**pod**: hexapod-mjx-train-3

**steps**: 40000000

**parent**: cw-arch-hist16

**hypothesis**: Mechanical retry of cw-arch-hist16 (identical config): prior attempt was a launch-collision EOFError (gotcha 13b, 0 steps), not a science result. Same TEMPORAL-ARCH rung-1 hypothesis (history_frames 8->16 on driving-champion config, from-scratch).

**gate**: det gait_valid 6/6 own-cfg DR0.5 + JOYSTICK GATE (eval_drive DR0.2) zero in-envelope falls + det prog_ratio med >=0.85 vs champion band; if zero gait emerges, verdict is 'bootstrap failure, history16 untested' NOT a history16 FAIL; frames watched det

**failed_reason**: run never appeared as 'running' in W&B within 240s

