# cw-dep-vref1-r1-fric-groundtilt5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T16:10:38+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**hardware_ready**: False

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: vref1-r1's two individually-PASSed floor-realism axes (friction scale 0.4-1.6x from -fric; 5deg ground tilt from -groundtilt5) have never been exposed TOGETHER, but a real floor is simultaneously imperfectly level AND has non-uniform grip -- the two conditions co-occur physically, unlike e.g. comshift (a body property). Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- floor-realism composes free like every other axis pairing tested tonight (comshift+deadband just launched this cycle). If-false: sloped+slick floor together defeats the contract-exact obs in a way neither did alone (plausible: tilt changes effective normal load exactly where friction margin is already thin) -- flag as a real hardware risk (uneven real flooring) before deployment.

**gate**: own-cfg (DR0.35 + friction0.4-1.6x + groundtilt5deg) det+sto 6/6 @15s gait_valid 12/12, 0 term, slip/m within vref1-r1's own band +-20%; DR0-no-override retention det 6/6 gv reproducing vref1-r1's own band; video frames watched det+sto

**verdict**: Own-cfg (DR0+friction0.4-1.6x+groundtilt5deg) det 5/6 ok gv 6/6 slip med 1.09, sto 6/6 ok gv 6/6 slip med 1.10 -- both inside vref1-r1's own combined band (0.89-1.36), 0 term either pass; 1 det crater is the lineage's known march-in-place fixed-draw stall (video-checked, no flag-leg/fall). Sloped + slick floor together still compose free, refutes the if-false interaction worry.

