# cw-walk-lowgait-dr035-latjit-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T01:32:11+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-walk-lowgait-dr035-latjit

**wandb_id**: fsf0jdk3

**hardware_ready**: False

**hypothesis**: 2nd launch attempt (1st died 0-step to fleet collision storm, gotcha 13b). Same hypothesis unchanged: crouch(-50mm, DR0.35 validated) x latency-jitter (0.5-2.5x) compose.

**gate**: Own-cfg harness DR0.35 (lowgait-dr035 cfg) + dr.latency_scale=0.5,2.5 det+sto 6/6 @15s: gait_valid 12/12, 0 term, mean end-height err<=10mm, slip/m<=1.6; DR0 nominal (no latency, no DR) retention det 6/6 gv, height err<=8mm, slip/m<=1.15; frames watched det for lurching/height overshoot

**verdict**: PASS. Latency-jitter (0.5-2.5x) composes cleanly onto the -50mm crouch + DR0.35 package. Own-cfg DR0.35+latency det+sto 6/6 gv, 0 term, height err mean 3.6mm det/3.4mm sto (gate<=10mm), slip/m med 1.13 det/1.32 sto (gate<=1.6), fwd med 0.73m det/0.65m sto @15s. DR0 nominal retention clean: gv 6/6, 0 term, height err mean 4.7mm (gate<=8mm), slip/m med 1.11 (gate<=1.15). Frames (det, own-DR): low stable crouch stance, all six legs cycling, no flag leg -- matches parent lowgait-dr035 exactly. One-two fixed-draw churn-tail episodes (low prog, high slip, no fall) in both own-DR and DR0 passes -- same canary-class pattern the parent already banked, not a new defect. Latency-jitter axis is free on the crouch line.

