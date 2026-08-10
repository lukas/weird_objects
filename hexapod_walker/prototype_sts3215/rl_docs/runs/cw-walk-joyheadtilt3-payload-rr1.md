# cw-walk-joyheadtilt3-payload-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T05:28:01+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-walk-joyheadtilt3

**wandb_id**: tmmflhnb

**hardware_ready**: False

**hypothesis**: Compose chassis payload (1.0-1.4x mass, the same range used everywhere else in the payload-compose ladder) onto this cycle's PASSed joyheadtilt3 package (+-90deg heading + DR0.5 + latency + 3deg floor slope) -- the next untried cell in the driving-package compose matrix (payload has composed free onto joyfric [friction-based] and joyheadfric [wide+friction], not yet tried on a wide+slope package). If-true: own-cfg (DR0.5+lat+tilt3+mass) det+sto 6/6 gv, 0 term, prog med matching joyheadtilt3's own band (~0.85/0.93), JOYSTICK GATE @90 0 falls, DR0 flat no-payload retention clean -- payload composability continues to track base-package headroom, not the specific axis composed. If-false: the slope+payload interaction (extra mass on a tilted stance) breaks something the friction composes didn't -- payload composability is package-specific after all, not just headroom-general.

**gate**: Own-cfg (DR0.5+lat+dr.ground_tilt_deg=3.0+dr.mass_scale=1.0,1.4) det+sto 6/6 @15s: gait_valid 6/6, 0 term, det prog med>=0.80 (joyheadtilt3's own band minus noise); JOYSTICK GATE @90deg 0 in-envelope falls; DR0 TRUE FLAT no-payload retention det 6/6 gv, prog med>=0.85, slip within noise of joyheadtilt3's own flat-retention band (slip 1.51/1.70); frames watched det

**verdict**: PASS: payload (1.0-1.4x mass) composes free onto the widest+slope driving package (joyheadtilt3: +-90deg heading + DR0.5 + latency + 3deg floor tilt). Own-cfg det gv 6/6 prog med 0.92 slip med 1.61, sto gv 6/6 prog med 0.92 slip med 1.69, 0 term either pass (gate: prog>=0.80). JOYSTICK GATE @90deg: 0 in-envelope falls across full direction panel + flip-stress. DR0 TRUE FLAT no-payload retention: det gv 6/6 prog med 0.92 slip med ~1.6, sto gv 6/6 prog med 0.92 slip med ~1.8, 0 sacrificed legs -- matches joyheadtilt3's own flat-retention band (1.51/1.70) within noise. Frames (own-cfg det, two episodes watched): six legs cycling swing/stance, no flag leg, no dragging, level-adjusted body -- known foot-slide/paddling persists unchanged from parent lineage, not new. Payload composability tracks base-package headroom (as with joyfric/joyheadfric), not the specific axis composed. Not hardware-ready (paddling gait; contract-line economics not yet applied to the driving lineage).

