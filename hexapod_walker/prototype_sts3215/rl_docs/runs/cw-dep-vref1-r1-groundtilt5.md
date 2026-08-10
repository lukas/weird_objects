# cw-dep-vref1-r1-groundtilt5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T07:30:57+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: fgxtphqp

**hardware_ready**: False

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: vref1-r1 has a wide safety.max_roll/pitch_deg=25 permission but has never trained on an actually-sloped floor (5deg, validated on the plain walk lineage) -- distinct from the tilt SAFETY envelope, this is a physical floor-slope DR axis. Real hardware ground (concrete/rug) is not perfectly flat. Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band; slope composes free like it does on every other walk lineage tonight. If-false: contract-exact velocity obs (meas:=ref) makes the honest-velocity gait less able to compensate for a persistent slope than legacy privileged obs -- flag before hardware.

**gate**: Own-cfg (DR0.35+groundtilt5deg) det+sto 6/6 @15s: gait_valid 6/6, 0 term, slip/m within vref1-r1's own band (~0.89-1.13 det, ~1.13-1.36 sto); DR0 flat retention clean; frames watched det

**verdict**: PASS -- 5deg floor slope composes onto the contract-exact hardware candidate (k_current=0 per P0). Own-cfg det+sto 6/6 gait_valid, 0 term, slip/m det med 0.98 / sto med 0.99 (within vref1-r1's own 0.89-1.13/1.13-1.36 band). Named-baseline retention pass (identical eval minus the slope cfg-set) gives clean det 6/6 med 0.94 fwd 0.77m (vs vref1-r1's own 0.89, within noise) and reproduces vref1-r1's OWN single-seed (idx4) sto-only paddling stall (prog 0.33 slip 4.50 vs vref1-r1's 0.26/5.97) -- checkpoint itself unchanged. Under the 5deg slope that same seed-4 draw's stall migrates into det too (prog~0.02 slip~27, fwd 0.13m) but stays a march-in-place (gait_valid True, no flag leg, level body -- confirmed on video det_0..5), matching the already-root-caused lineage paddling-attractor class (c79 dig-in), not new. Not hardware-ready on its own; clears ground-slope as a safe axis for the hardware candidate.

