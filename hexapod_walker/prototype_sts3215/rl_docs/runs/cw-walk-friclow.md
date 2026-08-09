# cw-walk-friclow

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T16:53:17+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: 0l7okvn2

**hardware_ready**: False

**hypothesis**: OPERATOR WISHLIST 13b, friction axis, slippery-floor half: real homes have tile/hardwood well below the sim's nominal grip, and the champion's DR band only reaches friction 0.6x. ISOLATED axis via dr.<field> overrides: dr-scale 0.0 with ONLY dr.friction_scale=0.3,1.0 randomized - one variable off the no-DR champion. If-true: the gait keeps transporting on slick ground (own-cfg gv 12/12, 0 term, det median fwd >=1.2m @30s) and DR0 retention holds - slippery floors join the envelope. If-false: propulsion collapses below ~0.6x grip (prog craters, slip/m explodes, feet skate in place on frames) - the paddle gait NEEDS grip, and slippery floors wait on the contact-pricing gait change like everything else. Strongest alternative: policy survives by slowing/parking rather than walking - fwd distance will show it.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.friction_scale=0.3,1.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2 m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24 (champion band); frames watched det

**verdict**: PASS on the letter, blunt caveats: grip 0.3-1.0x own-cfg gv 12/12, 0 term, det med fwd 1.23m (gate 1.2 — scrapes past); slip/m med 1.73 = heavy skating on slick draws, worst det draw 0.69m @ slip 3.79 / prog 0.37. DR0 retention gv 6/6 slip 1.01 BUT nominal det fwd eased 1.57->1.43 (ranges disjoint — real ~9% nominal cost, the only axis of today's three that charged one). Envelope: transports without falls down to 0.3x grip, but lowest-grip transport is skating, not walking-quality. NOT hardware-ready.

