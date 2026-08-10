# cw-arch-hist16-r6-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T03:12:41+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-arch-hist16-r5

**hardware_ready**: False

**hypothesis**: 6th mechanical retry of TEMPORAL-ARCH rung 1 (history_frames=16, from-scratch): base+r1..r5 all died at 0 steps to launch-collision EOFError, but ALL were direct launches that never got the self-repairing backlog retry-and-rename treatment (which just resolved an identically-symptomed 4x-failing spec, joyheaddeadband-s1-r3-c3, on its 5th auto-retry). Queuing via backlog this time so the drain's own collision self-repair can keep retrying instead of one manual shot each cycle. Same spec unchanged.

**gate**: det gait_valid 6/6 own-cfg DR0.5 + JOYSTICK GATE (eval_drive DR0.2) zero in-envelope falls + det prog_ratio med >=0.85 vs champion band; if zero gait emerges, verdict is 'bootstrap failure, history16 untested' NOT a history16 FAIL; frames watched det

**verdict**: ROOT CAUSE FOUND (deep dig-in 08-10, supersedes the launch-collision reading): the 8-attempt 0-step EOFError chain on this spec was /dev/shm exhaustion. Train pods have the 64M k8s default /dev/shm; a normal 4096-env sharded layout maps ~58M (measured on live healthy runs) and history_frames=16 doubles the obs block, pushing the layout past 64M — every worker SIGBUSes on first page touch (reproduced via instrumented smoke on train-7: all 24 workers exitcode -7). Architecture STILL untested — not a history16 FAIL. Fixes: worker faulthandler + exitcode reporting (c4f3625), startup GC of orphaned shm segments (bcf46be), dshm 4Gi volume in both pod manifests (recreate pods when idle). Retry queued as cw-arch-hist16-r7 at n-envs 3072 (~50M, fits current pods).

