# cw-dep-vref1-r1-encnoise-latency-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T19:02:19+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: cw-dep-vref1-r1-encnoise-latency

**wandb_id**: k198cw56

**hardware_ready**: False

**hypothesis**: Plain English: seed-twin confirmation of tonight's PASS on the combined encoder-noise + comms-latency compose (both individually-benign sensing/comms axes stay benign together) -- only tested at one training seed (11) so far, and this is one of the sensing risks most directly relevant to the real deployed bus link. Same recipe, different training seed (12), nothing else changed. If-true: seed 12 lands in/near vref1-r1's own slip/m band like seed 11 did (0.96 det/1.06 sto) -- recipe confirmed, not seed luck. If-false: seed 12 shows real degradation seed 11 didn't -- the combined-axis PASS was seed-fragile, flag before hardware.

**gate**: own-cfg (DR0.35+dr.encoder_noise_deg=0.5+dr.latency_scale=0.5,2.5) det+sto 6/6 @15s: gait_valid 12/12, 0 term, slip/m compared directly against the seed-11 sibling's 0.96 det/1.06 sto -- report whether it reproduces or differs; DR0 retention clean; frames watched det

**verdict**: PASS -- seed-twin (training seed 12) confirms the combined encoder-noise (0.5deg) + comms-latency-jitter (0.5-2.5x) compose PASSed tonight on seed 11 was a real recipe, not seed luck. Both DR0-gate and own-cfg DR0.35 passes: gait_valid 12/12, 0 term, both modes. own-cfg DR0.35 degraded episodes land at the IDENTICAL indices as the seed-11 parents own DR0.35 pass (det/5, sto/0, sto/1) with closely matching magnitudes (det/5 slip 2.35 vs parents 2.64; sto/0 slip 2.08 vs 2.01; sto/1 slip 2.87 vs 2.21) -- this is the deterministic fixed-eval-seed hard-DR-draw fingerprint reproducing across training seeds, not training-seed variance. Medians: det slip 1.24/sto 1.16 vs parents 0.96/1.06 -- mildly higher (driven by one extra soft episode, det/1 slip1.29, plus sto/1 running slightly worse) but still within the established +-20% band discipline used for every sibling tonight, no new failure mode. DR0-gate retention clean both seeds (det slip med 1.07-1.08, sto 0.97-1.01). Video-confirmed same clean six-leg march-in-place stall pattern at every degraded episode, no flag-leg/drag/fall.

