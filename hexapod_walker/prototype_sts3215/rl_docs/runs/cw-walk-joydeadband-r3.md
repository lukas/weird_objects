# cw-walk-joydeadband-r3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T00:17:00+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-joylat25

**hardware_ready**: False

**hypothesis**: 3rd retry of cw-walk-joydeadband (2 prior consecutive launch-collision losses; a 3rd backlog attempt disappeared mid-storm without a ledger trace -- requeuing once more). Same spec unchanged.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 --heading-max-deg 45 -- ZERO in-envelope falls; own-cfg (DR0.5+latency0.5-2.5x+deadband1-3x) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog_ratio med >=0.80; DR0 retention det 6/6 gv; frames watched det

**verdict**: Launch failure (gotcha 13b EOFError launch-collision at reset) — 0 steps trained, no science result. 4th consecutive launch-collision loss for this spec (base+r1+r2+r3x2).

**refused_reason**: W&B already has a run named cw-walk-joydeadband-r3 (names are append-only; pick a new one)

