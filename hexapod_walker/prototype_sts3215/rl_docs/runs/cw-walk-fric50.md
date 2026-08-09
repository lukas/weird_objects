# cw-walk-fric50

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T18:41:10+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: gg3z5m2e

**hardware_ready**: no

**hypothesis**: Envelope refinement after friclow's marginal PASS (grip 0.3-1.0x: transports but slick draws SKATE, slip/m med 1.73, and nominal fwd paid ~9%). One variable off the champion: dr.friction_scale=0.5,1.0 - the realistic tile/hardwood band, cutting the extreme 0.3x draws that forced skating. Plain: find the grip floor where walking stays CLEAN, not just survivable. If-true: moderate slip trains clean - own-cfg gv 12/12, 0 term, det med fwd >=1.3m AND own-cfg det slip/m med <=1.3 (near champion band, the clean-walking claim friclow failed) with DR0 retention at full champion speed (no 9% tax). If-false: even 0.5x grip forces the skating trade (own-cfg slip >1.5 or nominal cost repeats) - the paddle gait needs the contact-pricing fix for ANY reduced grip; close the friction axis until then. Strongest alternative: clean own-cfg but the nominal tax persists - then the tax is exposure-width-independent and worth flagging.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.friction_scale=0.5,1.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.3 m, det slip/m med <=1.3; plus DR0 nominal retention det 6/6 gv, det med fwd >=1.5 m, slip/m <=1.24; frames watched det

**verdict**: FAIL (pre-registered if-false): even the realistic 0.5-1.0x grip band forces the skating trade — own-cfg det slip/m med 1.35 (gate <=1.3), the 2 slickest det draws churn near-in-place (slip 3.06/3.50, fwd 0.62-0.69m vs 1.5m on grippy draws); gv 12/12, 0 term, det fwd med 1.33m scrapes the letter. Good news recorded honestly: DR0 retention is FULL champion speed (det fwd 1.57m, slip 0.86) — friclow's 9% nominal tax came from its 0.3x extreme draws, not from friction exposure per se. With friclow (slip 1.73) this is 2 misses on 'clean slick walking by exposure': friction-exposure axis CLOSED until the operator contact-pricing calibration (evidence: cw-walk-fric50 + cw-walk-friclow evals; slip root = contact pricing per plan). Paddle lineage, NOT hardware-ready.

