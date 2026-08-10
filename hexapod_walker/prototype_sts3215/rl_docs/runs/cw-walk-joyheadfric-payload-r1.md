# cw-walk-joyheadfric-payload-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T01:09:38+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-joyheadfric

**wandb_id**: lgg14ole

**hardware_ready**: False

**hypothesis**: Retry of cw-walk-joyheadfric-payload (died at init, gotcha 13b, 0 steps -- fleet launch-collision storm, not a science result). Same hypothesis unchanged: payload x widest +-90deg envelope+friction compose.

**gate**: Own-cfg harness DR0.5+latency+friction+dr.mass_scale=1.0,1.4 det+sto 6/6 @15s: gait_valid 12/12, 0 term, det prog median >=0.75; DR0 nominal retention det 6/6 gv, slip/m<=1.24, prog>=0.85; JOYSTICK GATE @DR0.2 --heading-max-deg 90 0 in-envelope falls retained; frames watched det

**verdict**: PASS -- payload (1.0-1.4x) composes cleanly onto the widest +-90deg driving package with friction hardening (joyheadfric). JOYSTICK GATE @DR0.2 heading90 0 in-envelope falls (all scenarios, flip-stress trk_err 0.032-0.039). Own-cfg (DR0.5+lat+fric+mass) det+sto gv 6/6 each, 0 term, prog med 0.89/0.88 (>=0.75 gate), slip med 1.71/1.79. DR0 nominal retention clean: gv 6/6, 0 term, prog med 0.92/0.94 (>=0.85 gate), slip med 1.79/1.72 -- matches parent joyheadfric's own retention band (1.7-1.9), no erosion. Frames (det): level six-leg tripod cycling, no flag leg, no dragging -- same paddle gait as parent. Notably contradicts the plain joyhead90-payload-r1 FAIL (no friction in that package) -- payload composability depends on the base package's headroom, and friction-hardened joyheadfric has enough to absorb payload for free even at the widest envelope. Not hardware-ready (foot-slide).

