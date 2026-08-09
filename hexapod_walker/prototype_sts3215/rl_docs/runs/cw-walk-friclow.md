# cw-walk-friclow

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T16:53:17+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: 0l7okvn2

**hypothesis**: OPERATOR WISHLIST 13b, friction axis, slippery-floor half: real homes have tile/hardwood well below the sim's nominal grip, and the champion's DR band only reaches friction 0.6x. ISOLATED axis via dr.<field> overrides: dr-scale 0.0 with ONLY dr.friction_scale=0.3,1.0 randomized - one variable off the no-DR champion. If-true: the gait keeps transporting on slick ground (own-cfg gv 12/12, 0 term, det median fwd >=1.2m @30s) and DR0 retention holds - slippery floors join the envelope. If-false: propulsion collapses below ~0.6x grip (prog craters, slip/m explodes, feet skate in place on frames) - the paddle gait NEEDS grip, and slippery floors wait on the contact-pricing gait change like everything else. Strongest alternative: policy survives by slowing/parking rather than walking - fwd distance will show it.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.friction_scale=0.3,1.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2 m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24 (champion band); frames watched det

