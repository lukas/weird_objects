# cw-walk-joytilt3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-09T20:43:46+00:00

**pod**: hexapod-mjx-train-7

**steps**: 20000000

**parent**: cw-walk-joylat25

**wandb_id**: r95a6tqc

**hardware_ready**: False

**hypothesis**: Driving-line compose off joylat25 (c60 best driving candidate), sibling of joyfric on a DIFFERENT axis. One variable: add dr.ground_tilt_deg=3.0 (the solid part of groundtilt5's c60 envelope - 5 deg was marginal, 3-4 deg clean). Plain: the operator will drive this across door thresholds and slightly sloped floors; steering with abrupt flips must not fall on a 3-degree slope. If-true: slope composes onto the driving package - joystick gate zero falls, own-cfg gv 12/12, prog med >=0.85, DR0 flat retention clean. If-false: downhill/uphill flips fall or crater progress - slope stays a straight-walk skill and the driving envelope is flat-floor only for now. Strongest alternative: no falls but heading drift downhill (lateral err grows vs parent) - check per-episode direction dists vs the parent joystick-gate baseline.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 own cfg - ZERO in-envelope falls; own-cfg (DR0.5 + dr.latency_scale=0.5,2.5 + dr.ground_tilt_deg=3.0) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog med >=0.85; DR0 flat retention det 6/6 gv; frames watched det

**verdict**: PASS - 3deg floor-slope axis composes cleanly onto the latency-hardened driving package (joylat25). JOYSTICK GATE @DR0.2 own cfg: 0 in-envelope falls (flip-stress trk_err 0.033-0.034). Own-cfg (DR0.5+latency0.5-2.5x+tilt3deg) det+sto: gait_valid 12/12, 0 term, prog med 0.94/0.98 (gate >=0.85), slip/m med 1.34/1.53. DR0 flat retention (no DR/tilt/latency): gv 12/12, prog med 1.01/1.01, slip 1.38/1.41 - matches champion driving band, no erosion from adding the slope axis. Frames (contact sheet, det): level body on flat eval floor, six legs cycling, no flag leg. Extends the driving envelope to include gentle floor slope (matches groundtilt5s solid 3-4deg sub-range); inherits paddle foot-slide (contact-pricing root) - not hardware-ready.

