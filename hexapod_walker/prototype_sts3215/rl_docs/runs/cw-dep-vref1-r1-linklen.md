# cw-dep-vref1-r1-linklen

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-10T17:02:43+00:00

**pod**: hexapod-mjx-train-7

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: real 3D-printed/CAD legs have print/assembly length error (global print-scale error AND independent per-leg per-segment spread) that vref1-r1 has never been exposed to -- the policy-side IK keeps NOMINAL leg lengths, so this is a genuine model/actuator mismatch axis, distinct from friction/mass/compliance already tested. Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- geometric print error composes free like the other structural axes (comshift, payload-mass FAILED but that was CoM/mass not length). If-false: leg-length mismatch breaks the fixed-foot IK's contact timing in a way no other axis did -- a real pre-attempt-#2 manufacturing-tolerance risk.

**gate**: own-cfg det+sto 6/6 @15s gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (0.89-1.36); frames watched det for flag-leg/skate

**refused_reason**: hexapod-mjx-train-7 code marker 32d8fb56026b423c906799a014056feaf82f9e6a != local HEAD e5406ed67980b8112e2b676562e488a337681c19. Sync first: snapshot.sh --sync hexapod-mjx-train-7 (and snapshot/commit before that if the tree is dirty).

