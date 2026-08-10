# cw-walk-joyheaddeadband-s1-r3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: KILLED

**created**: 2026-08-10T02:04:17+00:00

**pod**: hexapod-mjx-train-7

**steps**: 20000000

**parent**: cw-walk-joyheaddeadband-s1-r2

**wandb_id**: 15sjf9c2

**hardware_ready**: False

**hypothesis**: 3rd retry (2 consecutive infra losses already). Same spec unchanged: ruling-7 seed-1 twin of cw-walk-joyheaddeadband. If this also fails while neighbors succeed, flag for dig-in per the c67 joyhead90-lat25-s1 precedent (isolated-spec reset-choreography failure, not generic fleet storm).

**gate**: JOYSTICK GATE @90deg 0 in-envelope falls; own-cfg (DR0.5+lat+deadband1-3x) det+sto 6/6 @15s: gait_valid 6/6, 0 term, prog_ratio med>=0.80; DR0 retention gv 6/6; frames watched det.

**verdict**: INFRA, not science: killed early at 3.87M/20M steps (19%) when its node (g142d86) was host-starved this window (loadavg ~245/128, fps dropped 8961->5000); too undertrained to check against the gate. Continuation -c1 died 0-step to a launch-collision EOFError (gotcha 13b); -c2 retry (this cycle) died the same way. Matches the c67/c71-flagged joyheaddeadband-s1 isolated-spec pattern (6+ deaths across differently-named attempts) -- no verdict yet on the deadband-seed1 hypothesis itself, pending a clean run.

