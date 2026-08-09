# cw-walk-fric50

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T18:41:10+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: gg3z5m2e

**hypothesis**: Envelope refinement after friclow's marginal PASS (grip 0.3-1.0x: transports but slick draws SKATE, slip/m med 1.73, and nominal fwd paid ~9%). One variable off the champion: dr.friction_scale=0.5,1.0 - the realistic tile/hardwood band, cutting the extreme 0.3x draws that forced skating. Plain: find the grip floor where walking stays CLEAN, not just survivable. If-true: moderate slip trains clean - own-cfg gv 12/12, 0 term, det med fwd >=1.3m AND own-cfg det slip/m med <=1.3 (near champion band, the clean-walking claim friclow failed) with DR0 retention at full champion speed (no 9% tax). If-false: even 0.5x grip forces the skating trade (own-cfg slip >1.5 or nominal cost repeats) - the paddle gait needs the contact-pricing fix for ANY reduced grip; close the friction axis until then. Strongest alternative: clean own-cfg but the nominal tax persists - then the tax is exposure-width-independent and worth flagging.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.friction_scale=0.5,1.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.3 m, det slip/m med <=1.3; plus DR0 nominal retention det 6/6 gv, det med fwd >=1.5 m, slip/m <=1.24; frames watched det

