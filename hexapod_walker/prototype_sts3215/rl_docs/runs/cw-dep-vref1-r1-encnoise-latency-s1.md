# cw-dep-vref1-r1-encnoise-latency-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T19:02:19+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: cw-dep-vref1-r1-encnoise-latency

**hypothesis**: Plain English: seed-twin confirmation of tonight's PASS on the combined encoder-noise + comms-latency compose (both individually-benign sensing/comms axes stay benign together) -- only tested at one training seed (11) so far, and this is one of the sensing risks most directly relevant to the real deployed bus link. Same recipe, different training seed (12), nothing else changed. If-true: seed 12 lands in/near vref1-r1's own slip/m band like seed 11 did (0.96 det/1.06 sto) -- recipe confirmed, not seed luck. If-false: seed 12 shows real degradation seed 11 didn't -- the combined-axis PASS was seed-fragile, flag before hardware.

**gate**: own-cfg (DR0.35+dr.encoder_noise_deg=0.5+dr.latency_scale=0.5,2.5) det+sto 6/6 @15s: gait_valid 12/12, 0 term, slip/m compared directly against the seed-11 sibling's 0.96 det/1.06 sto -- report whether it reproduces or differs; DR0 retention clean; frames watched det

