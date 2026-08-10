# cw-dep-vref1-r1-zerobias

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T07:37:04+00:00

**pod**: hexapod-mjx-train-7

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: 4i0x6vje

**hardware_ready**: False

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: vref1-r1 has never trained with per-joint zero-point calibration bias (3deg, validated elsewhere) -- distinct from placement_noise (this is a fixed per-joint offset from imperfect servo-horn zeroing at assembly, not per-episode slop), directly relevant since the real robot's set_zero was done by hand. Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band; zero-bias composes free like it does elsewhere. If-false: a persistent per-joint offset interacts badly with the contract-exact obs (which assumes the reference is truthful) -- flag before hardware.

**gate**: Own-cfg (DR0.35+joint_zero_bias3deg) det+sto 6/6 @15s: gait_valid 6/6, 0 term, slip/m within vref1-r1's own band (~0.89-1.13 det, ~1.13-1.36 sto); DR0 no-bias retention clean; frames watched det

**verdict**: PASS -- joint zero-bias offset (3deg) composes free onto the contract-exact hardware-candidate checkpoint (k_current=0 per P0). Own-cfg (DR0.35+zerobias3deg) det 6/6 gv (prog med 0.98, slip med 1.06), sto 6/6 gv (prog med 1.04, slip med 1.00) -- both inside vref1-r1's own band (det slip 0.89, sto slip 1.13) within noise. One det ep (idx4) craters to the lineage's known march-in-place stall (prog -0.18, slip 27.55, fwd 0.12m -- same fixed-draw pattern as every other sibling compose, no flag-leg/fall). Not independently hardware-ready (inherits vref1-r1's own paddle-gait economics); clears 3deg joint zero-bias as a safe axis for the hardware candidate.

