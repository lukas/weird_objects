# cw-arch-hist24-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T15:24:01+00:00

**pod**: hexapod-mjx-train-5

**steps**: 40000000

**parent**: cw-arch-hist16-r7

**wandb_id**: 8mhvhqlv

**hypothesis**: Temporal-arch ladder rung 2 (plan queue 0.5, operator-reserved line; arch line at 1/2 allotted pods). hist16-r7 PASSED from scratch at 3072 envs (seed-confirmed r7-s1; continuation c1 training now): 640ms of history bootstraps and matches champion band. One variable: obs.history_frames 16->24 (~960ms at 25Hz), same from-scratch recipe (obs-width change forbids warm start; ent 0.01, std 1.0, 3072 envs, 40M). Prediction-if-true: bootstraps and matches/beats the hist16 band on joystick gate + det prog - more past state keeps helping, next rung is width control (256x256). Prediction-if-false: bootstrap fails or det prog degrades vs hist16 - 16 frames is the ladder plateau. Strongest alternative: trains but slower to gait within 40M - report steps-to-first-gait vs r7 before calling the rung.

**gate**: det gait_valid 6/6 own-cfg DR0.5 + JOYSTICK GATE (eval_drive DR0.2) zero in-envelope falls + det prog_ratio med >=0.85 vs champion band; if zero gait emerges verdict is 'bootstrap failure, history24 untested' NOT a history24 FAIL; frames watched det

