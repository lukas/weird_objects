# cw-dep-bcgait4-phasedir9-longrun17

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS (partial)

**created**: 2026-08-22T13:46:44+00:00

**pod**: hexapod-mjx-train-0

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-seed17

**hardware_ready**: False

**hypothesis**: Companion arm to longrun13, same budget-vs-mechanism question on the seed that did NOT near-pass at 2M: seed17 landed at/below pd8's level (0.727x progress, 1.27x slip) with reward still declining at cutoff, not obviously converged. Identical stack+seed(17) as phasedir9-seed17, ONLY --steps 2M->4M and --log-std-anneal-frac 0.6->0.3 (anneal still ends at the same absolute ~1.2M step; policy gets 2.8M steps at converged std=0.041 instead of ~800k). Prediction-if-true: seed17 recovers toward or past pd9's 2M numbers with the extra converged-regime budget -- seed17's bad 2M reading was itself undertrained, not a worse basin. Prediction-if-false: seed17 stays flat/worse through 4M despite the same schedule that (maybe) helps seed13 -- real seed-dependent basin selection at this budget, independent of anneal timing, which redirects investment toward the phase-lock/anchor dig-in and away from budget scaling for either seed.

**gate**: Same clone-relative forward panel as pd9 (logs/ckpt_eval/phasedir3_clone_control_gate, DR-0 det+sto). Report progress/slip/speed/falls at final (4M) checkpoint plus the ~2M mid-run eval for trend. PASS = zero falls, gait_valid 6/6, progress >=0.9x clone, slip <=1.15x clone, speed in [0.06,0.096]. Compare directly against this run's OWN 2M reading (0.727x/1.27x) and longrun13's paired result before drawing a lineage-wide conclusion.

**verdict**: CORRECTION (the two prior commits' numbers do not match the synced eval — see below): the committed verdict (0.719x prog/1.32x slip/0.058 speed, FAIL) was written at 14:03:08 and 14:07:02, BEFORE/without using the actual synced report (gate report.json finished syncing at 14:03:45, owncfg at 14:04:04; W&B eval/dr0/walk_det/* summary keys match the on-disk report exactly). Recomputed from the real synced data (logs/ckpt_eval/cw_dep_bcgait4_phasedir9_longrun17_{gate,owncfg}/report.json, cross-checked against W&B run iggpxkeb summary and re-run via ops.sh report): DR-0 DET progress_ratio med 0.784 (clone 0.766-0.77 -> ratio 1.02x, cap >=0.9x PASS), slip_per_m med 1.406 (clone 1.887-1.89 -> ratio 0.74x, cap <=1.15x PASS), speed med 0.069 m/s (in [0.06,0.096] PASS), zero falls/terminations across all 24 det+sto episodes, gait_valid 6/6, roll_class clean all 6, no sacrificed legs, dir_err_mean 27-29deg (better than the ~35deg clone floor). Own-DR (0.35) DET also clears all three: progress 0.72 raw -> 0.94x (thin but >=0.9x), slip 1.91 raw -> 1.01x (<=1.15x), speed 0.067. This is the FIRST full clone-relative rung-A gate PASS (det, both DR-0 and own-DR) anywhere in the phasedir1-9/longrun lineage (34+ prior arms), paired with longrun13 (same recipe, seed13) which genuinely regressed (0.792x/1.286x) -- a real seed-dependent divergence, not a uniform budget-helps-or-not answer. STO still fails both conditions (prog ~0.65x, slip ~1.76x), consistent with the lineage-wide det/sto gap (the clone's own sto baseline is itself degenerate, prog 0.14/slip 23.6, so sto was never part of the clone-relative ratio criteria). Video (contact sheet + frame-tiled walk_det_0.mp4) shows clean 6-leg alternating gait, forward translation, no drag/flag-leg/freeze. Marked PASS (partial) not a full lineage-closing PASS pending DIG-IN: (a) own-DR progress margin is thin (0.94x), (b) sto remains bad, (c) the seed13-vs-seed17 divergence under an identical recipe needs a root-cause pass (does not by itself relaunch the budget-scaling lever generally -- see STATUS/CURRENT_TRUTHS correction) before any promotion/champion-append or further budget-arm spend.

