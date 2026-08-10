# cw-walk-lowgait-dr035-latjit-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T01:32:11+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-walk-lowgait-dr035-latjit

**wandb_id**: fsf0jdk3

**hypothesis**: 2nd launch attempt (1st died 0-step to fleet collision storm, gotcha 13b). Same hypothesis unchanged: crouch(-50mm, DR0.35 validated) x latency-jitter (0.5-2.5x) compose.

**gate**: Own-cfg harness DR0.35 (lowgait-dr035 cfg) + dr.latency_scale=0.5,2.5 det+sto 6/6 @15s: gait_valid 12/12, 0 term, mean end-height err<=10mm, slip/m<=1.6; DR0 nominal (no latency, no DR) retention det 6/6 gv, height err<=8mm, slip/m<=1.15; frames watched det for lurching/height overshoot

