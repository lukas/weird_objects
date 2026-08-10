# cw-walk-joyhead90-payload-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T00:21:37+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-walk-joyhead90-payload

**wandb_id**: e7t9hn7k

**hardware_ready**: False

**hypothesis**: Retry of cw-walk-joyhead90-payload (lost the fleet launch-collision race, gotcha 13b, 0 steps trained, no science result). Same spec unchanged: chassis-payload exposure (dr.mass_scale=1.0,1.4x) composed onto the widest driving package (joyhead90-lat25: +-90deg abrupt flips + DR0.5 + latency jitter). Tests whether payload composability generalizes to the wide envelope, distinct from the narrower joyfric-payload base.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 --heading-max-deg 90 own cfg - ZERO in-envelope falls; own-cfg DR0.5+latency+dr.mass_scale=1.0,1.4 det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog_ratio med >=0.75; DR0 nominal retention det 6/6 gv, slip/m<=1.24, prog>=0.9; frames watched det

**verdict**: FAIL (retention letter missed) — payload/mass exposure (1.0-1.4x) composed onto the widest +-90deg driving envelope (joyhead90-r1: abrupt flips + DR0.5 + latency 0.5-2.5x). Own-cfg (DR0.5+lat+mass) harness det+sto 6/6 @15s: gv 12/12, 0 term, prog med 0.92/0.90 (>=0.75 gate) -- PASSES own-cfg. But pre-registered DR0 nominal retention (no payload/mass override) misses on BOTH thresholds: slip/m med 1.31 det / 1.43 sto (cap 1.24), prog med 0.89/0.89 (gate >=0.90). Frames det: no new pathology -- level body, six legs still cycling, same paddle foot-slide as the parent, no flag leg/falls. This matches the established payload-dr05 FAIL class (c61): composing a payload/mass DR band onto an already DR-trained steering package charges nominal-floor tracking by construction, not a gait breakdown. Did not run the eval_drive JOYSTICK GATE since the retention leg already fails the letter -- would not change the verdict. Not hardware-ready. No requeue: this closes payload-on-steering-package composability as charged-nominal, same root cause as payload-dr05, now confirmed on a 2nd (wider) driving package.

