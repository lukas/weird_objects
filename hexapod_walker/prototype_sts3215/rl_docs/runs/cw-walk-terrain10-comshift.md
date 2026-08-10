# cw-walk-terrain10-comshift

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T04:58:17+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-terrain10

**hardware_ready**: False

**hypothesis**: NEW compose, untried pairing: off-center CoM payload shift (0.03m, the comshift envelope PASSed onto the plain champion, groundtilt5, and multiple driving packages) x rough-ground terrain hardening (terrain10, hfield bumps to 36mm, PASS/SATURATED -- bumps never perturb the paddle gait). Terrain has not been tried with an off-axis CoM load. If-true: own-cfg (terrain10+comshift) det+sto 6/6 gv, 0 term, prog med matching terrain10's own band (~1.0-1.06); DR0 flat-no-bumps-no-offset retention clean. If-false: the off-axis load interacts with the bump perturbations to crater progress or cost falls that flat-ground comshift composes did not show.

**gate**: Own-cfg (dr.terrain_amp=1.0 + dr.com_offset_m=0.03) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det prog med>=0.85; DR0 flat-no-bumps-no-offset retention det 6/6 gv, slip/m<=1.24; frames watched det

**verdict**: PASS: rough-terrain(amp1.0) x off-center CoM compose holds. Own-cfg det/sto prog med 1.01/1.01 (>=0.85 gate), slip med 1.08/0.99, gv 12/12, 0 term; DR0 flat-no-bumps-no-offset retention det/sto prog med 1.02/1.04, slip med 1.05/0.97 (<=1.24 gate), gv 12/12, 0 term. One draw (det/sto idx4) crashes to prog 0.06-0.07/slip 23-25 in BOTH the terrain+comshift config AND the flat/no-offset retention config identically -- confirmed via report.json (terminated=false, sacrificed_legs=[], all 6 legs swinging 11-20x, along_dist 0.04m vs cmd_dist 0.69m) as a march-in-place stall intrinsic to this checkpoint/draw, NOT caused by terrain or comshift (same draw, same magnitude, with or without either). Video on both configs: level body, six legs cycling, no flag leg/fall. Terrain and off-axis load compose free; the one bad draw is a known lineage-class fixed-draw stall, gate still met on medians.

**refused_reason**: hexapod-mjx-train-0 already runs cw-walk-terrain10-comshift — GPU pods host exactly one run; pick a free GPU pod.

