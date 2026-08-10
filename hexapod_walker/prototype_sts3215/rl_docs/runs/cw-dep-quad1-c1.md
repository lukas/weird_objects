# cw-dep-quad1-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-10T17:28:40+00:00

**pod**: hexapod-mjx-train-0

**steps**: 12000000

**parent**: cw-dep-quad1

**hypothesis**: Plain English: does the four-leg party trick just need more training time on the hardware-realistic checkpoint, or is it structurally stuck? cw-dep-quad1 FAILED the height-control gate (height_err_end_mm plateaued at 31mm vs the <=20mm gate) but was STILL improving at the end of its 18M steps (60->31mm, monotonic, no sign of a hard plateau in the last 3 points: 34->33->31mm) and never fell (survived_frac 1.0 throughout) -- one variable vs quad1: +12M more steps warm-started from quad1's own checkpoint, same mix/reward, nothing else changed. If-true: height_err_end_mm continues dropping and clears <=20mm within the extension -- quad1 was just under-trained, not structurally blocked by the missing privileged-velocity signal. If-false: height_err_end_mm plateaus above 20mm despite the extra steps -- confirms the mechanism is capped by the honest-obs contract itself, not training budget, and the quad line should stay on the privileged-velocity quad-hold2 checkpoint.

**gate**: Quad-mode training eval: height_err_end_mm <=20mm at the final 2 logged checkpoints (not just one), survived_frac >=0.9 maintained, track_err_deg does not keep climbing; own-cfg walk-mode det gv 6/6, 0 term, slip/m med <=1.35 (retention unaffected by the extra steps)

**refused_reason**: hexapod-mjx-train-0 code marker f634093caa6dca968acb22b59d1df41b6af8fa91-dirty != local HEAD f634093caa6dca968acb22b59d1df41b6af8fa91. Sync first: snapshot.sh --sync hexapod-mjx-train-0 (and snapshot/commit before that if the tree is dirty).

