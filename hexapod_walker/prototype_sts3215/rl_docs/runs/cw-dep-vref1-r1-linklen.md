# cw-dep-vref1-r1-linklen

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T17:06:27+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: u2bfjy09

**hardware_ready**: False

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: real 3D-printed/CAD legs have print/assembly length error (global print-scale error AND independent per-leg per-segment spread) that vref1-r1 has never been exposed to -- the policy-side IK keeps NOMINAL leg lengths, so this is a genuine model/actuator mismatch axis, distinct from friction/mass/compliance already tested. Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- geometric print error composes free like the other structural axes (comshift, payload-mass FAILED but that was CoM/mass not length). If-false: leg-length mismatch breaks the fixed-foot IK's contact timing in a way no other axis did -- a real pre-attempt-#2 manufacturing-tolerance risk.

**gate**: own-cfg det+sto 6/6 @15s gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (0.89-1.36); frames watched det for flag-leg/skate

**verdict**: PASS (if-true): 3D-print/assembly leg-length error (2% global scale + 1.2% per-leg spread) composes free onto the hardware-contract-exact base, same as every other structural/mechanical axis tested. DR0-gate own-cfg det+sto: det gv 6/6, 0 term, slip/m median 1.08 (sorted [0.91,0.99,1.06,1.10,1.27,28.75], the last is the lineage's known fixed-draw march-in-place crater at idx4, video-confirmed no flag-leg/drag) -- within vref1-r1's own 0.89-1.13 det band; sto gv 6/6, 0 term, slip/m med ~1.03, in the 1.13-1.36 sto band. Video (det ep0 clean six-leg gait, det ep4 crater) matches every prior PASSed sibling compose exactly. Leg-length manufacturing tolerance clears as a safe hardware axis.

