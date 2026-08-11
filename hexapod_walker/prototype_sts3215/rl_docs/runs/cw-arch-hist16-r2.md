# cw-arch-hist16-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T00:54:29+00:00

**pod**: hexapod-mjx-train-3

**steps**: 40000000

**parent**: cw-arch-hist16-r1

**hypothesis**: 3rd mechanical attempt (2 straight launch-collision EOFErrors, gotcha 13b, 0 steps each — still not a science result). Same TEMPORAL-ARCH rung-1 spec unchanged.

**gate**: det gait_valid 6/6 own-cfg DR0.5 + JOYSTICK GATE (eval_drive DR0.2) zero in-envelope falls + det prog_ratio med >=0.85 vs champion band; if zero gait emerges, verdict is 'bootstrap failure, history16 untested' NOT a history16 FAIL; frames watched det

**verdict**: no verdict on this arm itself (0 steps, launch never reached "running" in W&B — a mechanical launch-collision failure, not a science result); class stopped by cw-arch-hist16-r6-rr1 dig-in, which found the actual root cause: /dev/shm exhaustion at n-envs=4096 + history_frames=16 (obs block doubling pushes past the 64M k8s default, SIGBUS on first page touch). Fixed (dshm volume + shm GC) and retried at n-envs=3072 as cw-arch-hist16-r7, which PASSED. No action needed on this entry beyond this note.

**failed_reason**: run never appeared as 'running' in W&B within 240s

