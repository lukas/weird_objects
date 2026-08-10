# cw-arch-hist16-r4

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T01:54:53+00:00

**pod**: hexapod-mjx-train-10

**steps**: 40000000

**parent**: cw-arch-hist16-r3

**hypothesis**: 5th mechanical retry of TEMPORAL-ARCH rung 1 (history_frames=16, from-scratch): base+r1+r2+r3 all died at 0 steps to launch-collision EOFError (gotcha 13b), flagged DIG-IN for possible from-scratch/sharded-env interaction but fleet load has eased since (350->110-230); trying once more mechanically before further blind retries. Same spec unchanged.

**gate**: det gait_valid 6/6 own-cfg DR0.5 + JOYSTICK GATE (eval_drive DR0.2) zero in-envelope falls + det prog_ratio med >=0.85 vs champion band; if zero gait emerges, verdict is 'bootstrap failure, history16 untested' NOT a history16 FAIL; frames watched det

**failed_reason**: run never appeared as 'running' in W&B within 240s

