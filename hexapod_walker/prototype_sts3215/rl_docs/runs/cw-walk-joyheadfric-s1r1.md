# cw-walk-joyheadfric-s1r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-10T00:15:01+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-joyheadfric

**wandb_id**: 9rx8zntx

**hardware_ready**: False

**hypothesis**: Retry of cw-walk-joyheadfric-s1 (lost a launch-collision race, gotcha 13b). Same seed-1 spec unchanged -- queued for the passive/watcher drain.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 --heading-max-deg 90 -- ZERO in-envelope falls; own-cfg (DR0.5+latency+friction) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog_ratio med >=0.80; DR0 retention det 6/6 gv; frames watched det

**verdict**: PASS -- seed-1 twin confirms cw-walk-joyheadfric (widest +-90deg envelope + friction 0.4-1.6x) is recipe-robust, not seed luck. JOYSTICK GATE @90deg 0 in-envelope falls (fwd/left/right/diag dist 0.21-0.30m, flip-stress trk_err 0.027-0.037). Own-cfg (DR0.5+friction) det+sto gv 6/6 each, 0 term, prog med 0.95/0.94 (>=0.80 gate), slip med 1.55/1.76. DR0 nominal retention clean: gv 6/6, 0 term, prog med 0.94/0.93 -- matches seed-0 band, no erosion. Frames (det): level six-leg tripod cycling, no flag leg, no dragging, no leg-through-floor -- same paddle gait as parent, still not hardware-ready (foot-slide).

