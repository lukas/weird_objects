# cw-dep-quad1-c2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T17:31:40+00:00

**pod**: hexapod-mjx-train-0

**steps**: 12000000

**parent**: cw-dep-quad1-c1

**wandb_id**: wo8twcy8

**hardware_ready**: False

**hypothesis**: Plain English: does the four-leg party trick just need more training time on the hardware-realistic checkpoint, or is it structurally stuck? cw-dep-quad1 FAILED the height-control gate (height_err_end_mm plateaued at 31mm vs the <=20mm gate) but was STILL improving at the end of its 18M steps (60->31mm, monotonic, no sign of a hard plateau in the last 3 points: 34->33->31mm) and never fell (survived_frac 1.0 throughout) -- one variable vs quad1: +12M more steps warm-started from quad1's own checkpoint, same mix/reward, nothing else changed. Re-queued as -c2 after -c1 was mechanically REFUSED/parked (pod conflict, then a stale-dirty code-marker bug in snapshot.sh, now fixed) -- same hypothesis, no science change. If-true: height_err_end_mm continues dropping and clears <=20mm within the extension -- quad1 was just under-trained, not structurally blocked by the missing privileged-velocity signal. If-false: height_err_end_mm plateaus above 20mm despite the extra steps -- confirms the mechanism is capped by the honest-obs contract itself, not training budget, and the quad line should stay on the privileged-velocity quad-hold2 checkpoint.

**gate**: Quad-mode training eval: height_err_end_mm <=20mm at the final 2 logged checkpoints (not just one), survived_frac >=0.9 maintained, track_err_deg does not keep climbing; own-cfg walk-mode det gv 6/6, 0 term, slip/m med <=1.35 (retention unaffected by the extra steps)

**verdict**: PASS (if-true confirmed): more training does resolve quad1's height-control gap, it was under-trained, not structurally capped by the honest-obs contract. Training curve: eval/quad/height_err_end_mm falls monotonically through the extension and clears the <=20mm gate at BOTH final logged checkpoints (10.0M step 3.7mm, 12.0M step 2.86mm, vs quad1's own plateau at 31-60mm over its whole run); survived_frac 1.0 throughout, track_err_deg keeps improving (1.75->1.49deg), not climbing. Walk-mode retention: own-cfg det gv 6/6, 0 term, slip/m med 1.17 (<=1.35 gate), video clean six-leg gait both a clean episode and the lineage's known march-in-place fixed-draw crater (no flag-leg/drag). Caveat (not gating, det is): sto walk retention hit ONE genuine flag-leg episode (sacrificed_legs [3,5], gv 5/6) distinct from the usual march-in-place-only crater -- worth a second look if this checkpoint becomes a base for further quad composes.

