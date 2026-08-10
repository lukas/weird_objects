# cw-dep-vref1-r1-fric-groundtilt5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-10T16:09:26+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: vref1-r1's two individually-PASSed floor-realism axes (friction scale 0.4-1.6x from -fric; 5deg ground tilt from -groundtilt5) have never been exposed TOGETHER, but a real floor is simultaneously imperfectly level AND has non-uniform grip -- the two conditions co-occur physically, unlike e.g. comshift (a body property). Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- floor-realism composes free like every other axis pairing tested tonight (comshift+deadband just launched this cycle). If-false: sloped+slick floor together defeats the contract-exact obs in a way neither did alone (plausible: tilt changes effective normal load exactly where friction margin is already thin) -- flag as a real hardware risk (uneven real flooring) before deployment.

**gate**: own-cfg (DR0.35 + friction0.4-1.6x + groundtilt5deg) det+sto 6/6 @15s gait_valid 12/12, 0 term, slip/m within vref1-r1's own band +-20%; DR0-no-override retention det 6/6 gv reproducing vref1-r1's own band; video frames watched det+sto

**refused_reason**: hexapod-mjx-train-4 already runs cw-dep-startvar1-noZDnoBS1 — GPU pods host exactly one run; pick a free GPU pod.

