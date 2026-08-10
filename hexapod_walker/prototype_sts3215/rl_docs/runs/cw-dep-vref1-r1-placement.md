# cw-dep-vref1-r1-placement

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T07:21:24+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: crm1q3jm

**hardware_ready**: False

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: vref1-r1 (contract-exact obs + 25deg tilt) PASSed and is the leading checkpoint for tonight's hardware attempt #2, but it has never been exposed to per-joint hand-placement/assembly slop (6deg, the validated PASS level from placementnoise6). Real servo horns get hand-assembled slightly off-center; if contract-exact velocity obs interacts badly with that slop (the estimator sees honest ref velocity but the joint's actual zero is offset), that matters more for hardware than any sim metric. Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band; placement noise composes free like it did on the plain champion. If-false: contract-exact obs + placement slop interact badly (new failure mode) -- flag before hardware.

**gate**: Own-cfg (DR0.35+placement6deg) det+sto 6/6 @15s: gait_valid 6/6, 0 term, slip/m within vref1-r1's own band (~0.89-1.13 det, ~1.13-1.36 sto); DR0 no-noise retention clean; frames watched det

**verdict**: PASS -- hand-placement/joint-placement noise 6deg composes free onto the contract-exact hardware-candidate checkpoint (k_current=0 per P0). Own-cfg (DR0.35+placement6deg) det 6/6 gv (prog med 0.99, slip med 1.09), sto 6/6 gv (prog med 1.01, slip med 1.04) -- both inside vref1-r1's own band (det slip 0.89, sto slip 1.13) within noise. One det ep (idx4) craters to the lineage's known march-in-place stall (prog 0.05, slip 25.79, fwd 0.19m -- frame-checked: level body, all 6 legs still cycling, no flag-leg/fall), same fixed-draw pattern as every other sibling compose (fric/comshift/deadband/encnoise/groundtilt5/gyronoise/imumount/latency). Not independently hardware-ready (inherits vref1-r1's own paddle-gait economics); clears 6deg placement noise as a safe axis for the hardware candidate.

