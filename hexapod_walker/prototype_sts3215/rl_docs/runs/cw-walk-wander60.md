# cw-walk-wander60

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-09T16:05:52+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: cw-walk-wander30

**hypothesis**: Driving endurance rung 2: wander30 PASSED 30 s drives (~6 command changes/ep, prog 0.94-1.02, zero stalls) and straight-line endurance holds at 60 s (endur60 PASS, no decay). One variable off wander30: horizon 30→60 s (~12 changes/ep). Plain: prove command-following doesn't decay over long joystick sessions. If-true: 60 s eps hold prog ~1.0 with gv and no term through all changes — drive duration is a non-issue. If-false: degradation accumulates only under steering (transition fatigue) — height sag/slip growth/parked segments after later changes.

**gate**: own-cfg DR0 60s det+sto 6/6: gait_valid 12/12, zero terminations, prog_ratio median 0.85-1.15, no ep prog<0.5; frames watched det

**refused_reason**: hexapod-mjx-train-10 code marker 920e6331c810f772b2bf80090ad921361f7d86de-dirty != local HEAD 920e6331c810f772b2bf80090ad921361f7d86de. Sync first: snapshot.sh --sync hexapod-mjx-train-10 (and snapshot/commit before that if the tree is dirty).

