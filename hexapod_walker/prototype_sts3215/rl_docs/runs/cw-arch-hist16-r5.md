# cw-arch-hist16-r5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T02:21:48+00:00

**pod**: hexapod-mjx-train-3

**steps**: 40000000

**parent**: cw-arch-hist16-r4

**hypothesis**: 6th mechanical retry of TEMPORAL-ARCH rung 1 (history_frames=16, from-scratch): base+r1+r2+r3+r4 all died at 0 steps to the launch-collision EOFError (gotcha 13b) during an extremely heavy multi-cycle drain-storm window (01:00-02:15, many concurrent cycles racing free pods + a self-repair dedup bug this cycle that silently dropped same-name requeues without parking -- launch_run.py rr-rename fix landed, snapshot 178fc8a). Operator directive (08-09 evening): keep 1-2 pods on the temporal-arch line whenever it is empty. Trying once more under a fresh name; if this ALSO dies to the identical EOFError, flag DIG-IN per the earlier note (possible from-scratch/sharded-env reset-choreography interaction specific to this spec, not generic fleet storm) rather than a 7th blind retry.

**gate**: det gait_valid 6/6 own-cfg DR0.5 + JOYSTICK GATE (eval_drive DR0.2) zero in-envelope falls + det prog_ratio med >=0.85 vs champion band; if zero gait emerges, verdict is 'bootstrap failure, history16 untested' NOT a history16 FAIL; frames watched det

**failed_reason**: launch-collision EOFError at first env reset (same pattern as gotcha 13b) -- log shows Traceback/EOFError right after warm start; this attempt landed on train-3 within ~1 min of another cycle's launch (cw-walk-joyheaddeadband-s1-r3-c1) hitting train-3 at nearly the same timestamp, i.e. two launches racing onto the SAME pod slot, not just neighbor-pod contention. 5th consecutive infra loss for this line-name. 0 steps, no science.

