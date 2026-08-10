# cw-dep-vref1-r1-fric-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T19:03:35+00:00

**pod**: hexapod-mjx-train-7

**steps**: 8000000

**parent**: cw-dep-vref1-r1-fric

**wandb_id**: nizxmzu5

**hypothesis**: Plain English: seed-twin confirmation of tonight's PASS-with-caveat friction compose (0.4-1.6x floor friction on the hardware candidate) -- the first seed (11) showed a det slip/m median (1.23) sitting ~9%% over vref1-r1's own upper band edge, inside tolerance but flagged so it isn't silently averaged away. This run reproduces the identical recipe with a different training seed (12) to check whether the elevated median is a friction-axis property (should reproduce) or seed-11 training luck (should not). If-true: seed 12 lands in/near the same elevated-but-in-tolerance band -- friction axis confirmed as a real, mild, tolerable cost, not seed noise. If-false: seed 12 lands cleanly in vref1-r1's own nominal band -- the elevated seed-11 median was training-seed luck, not a friction-driven effect.

**gate**: own-cfg (DR0.35+dr.friction_scale=0.4,1.6) det+sto 6/6 @15s: gait_valid 12/12, 0 term, slip/m compared directly against the seed-11 sibling's 1.23 det/1.04 sto -- report whether it reproduces (within ~15%%) or differs; DR0 retention clean; frames watched det

