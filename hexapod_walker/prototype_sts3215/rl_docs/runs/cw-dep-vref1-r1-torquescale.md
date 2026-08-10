# cw-dep-vref1-r1-torquescale

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T15:50:09+00:00

**pod**: hexapod-mjx-train-6

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**hypothesis**: respec of cw-dep-vref1-r1: PROTECT THE NAMED HARDWARE CANDIDATE. vref1-r1 has only ever seen the DEFAULT torque_scale range (0.80-1.05x, baked into every DR>0 run -- the prior 'torque-droop' closed-axis composes on OTHER lineages tested exactly this same default range, not a widened one, so they are uninformative here). This arm widens the low end (0.5-1.05x) to represent real battery-sag/servo-spread under load, distinct from the already-closed default-range axis. Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- deeper torque sag composes free like every other axis so far (comshift/deadband/gyronoise/imumount/latency/placement/zerobias/encnoise/groundtilt5/friction all PASSed; payload/mass is the one axis that FAILED). If-false: reduced available torque under the contract-exact obs breaks the gait (plausible: less margin to correct slip/stumbles with a weaker actuator) -- flag as a real hardware risk (battery voltage sags under load) before deployment.

**gate**: own-cfg (DR0.35+torque0.5-1.05) det+sto 6/6 @15s gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (det 0.89-1.13, sto 1.13-1.36) +-20% tolerance; DR0 no-torque-override retention det 6/6 gv reproducing vref1-r1's own band; video frames watched det+sto

