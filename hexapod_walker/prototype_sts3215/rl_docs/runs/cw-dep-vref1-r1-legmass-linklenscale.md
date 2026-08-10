# cw-dep-vref1-r1-legmass-linklenscale

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T19:29:36+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: cw-dep-vref1-r1-legmass

**wandb_id**: 4porxyoa

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: vref1-r1's per-leg manufacturing-tolerance axis (leg-mass jitter 0.20 + per-leg link-length spread 0.025, PASSED as -legmass) has never been stacked with a GLOBAL print-shrinkage scale (link_len_scale_pct=0.02, PASSED alone as -linklen) -- a real 3D-printed leg batch realistically has both a uniform shrinkage/scale error AND per-leg random variation AND mass jitter at once, not one at a time. Per P0 rule 3, k_current=0 (inherited). If-true: own-cfg (all 3 length/mass axes) det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- the manufacturing-tolerance stack composes free like every other axis pairing tonight. If-false: global scale error compounds with per-leg jitter (both change effective leg length, unlike most other pairings which touch independent physical quantities) enough to break tracking -- flag as a real pre-attempt-#2 assembly-QA risk.

**gate**: own-cfg (DR0.35 + dr.leg_mass_jitter_pct=0.20 + dr.link_len_leg_pct=0.025 + dr.link_len_scale_pct=0.02) det+sto 6/6 @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (0.89-1.36); DR0 retention clean; frames watched det

