# cw-walk-terrain10-comshift-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: KILLED

**created**: 2026-08-10T05:00:54+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-terrain10

**wandb_id**: ikbaemoz

**hypothesis**: NEW compose, untried pairing: off-center CoM payload shift (0.03m, the comshift envelope PASSed onto the plain champion, groundtilt5, and multiple driving packages) x rough-ground terrain hardening (terrain10, hfield bumps to 36mm, PASS/SATURATED -- bumps never perturb the paddle gait). Terrain has not been tried with an off-axis CoM load. If-true: own-cfg (terrain10+comshift) det+sto 6/6 gv, 0 term, prog med matching terrain10's own band (~1.0-1.06); DR0 flat-no-bumps-no-offset retention clean. If-false: the off-axis load interacts with the bump perturbations to crater progress or cost falls that flat-ground comshift composes did not show.

**gate**: Own-cfg (dr.terrain_amp=1.0 + dr.com_offset_m=0.03) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det prog med>=0.85; DR0 flat-no-bumps-no-offset retention det 6/6 gv, slip/m<=1.24; frames watched det

**verdict**: Killed as a duplicate: the drain's own self-repair rename raced and landed BOTH cw-walk-terrain10-comshift (train-0, @5.5M steps) and this -rr1 retry (train-1, @3M steps) as live training pods for the identical respec spec. Kept the more-progressed original, killed this one to free the GPU pod rather than waste compute on an exact duplicate; 0 science lost (real result comes from the surviving cw-walk-terrain10-comshift).

