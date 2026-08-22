# cw-dep-bcgait4-phasedir9-longrun17

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-22T13:46:44+00:00

**pod**: hexapod-mjx-train-0

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-seed17

**hardware_ready**: False

**hypothesis**: Companion arm to longrun13, same budget-vs-mechanism question on the seed that did NOT near-pass at 2M: seed17 landed at/below pd8's level (0.727x progress, 1.27x slip) with reward still declining at cutoff, not obviously converged. Identical stack+seed(17) as phasedir9-seed17, ONLY --steps 2M->4M and --log-std-anneal-frac 0.6->0.3 (anneal still ends at the same absolute ~1.2M step; policy gets 2.8M steps at converged std=0.041 instead of ~800k). Prediction-if-true: seed17 recovers toward or past pd9's 2M numbers with the extra converged-regime budget -- seed17's bad 2M reading was itself undertrained, not a worse basin. Prediction-if-false: seed17 stays flat/worse through 4M despite the same schedule that (maybe) helps seed13 -- real seed-dependent basin selection at this budget, independent of anneal timing, which redirects investment toward the phase-lock/anchor dig-in and away from budget scaling for either seed.

**gate**: Same clone-relative forward panel as pd9 (logs/ckpt_eval/phasedir3_clone_control_gate, DR-0 det+sto). Report progress/slip/speed/falls at final (4M) checkpoint plus the ~2M mid-run eval for trend. PASS = zero falls, gait_valid 6/6, progress >=0.9x clone, slip <=1.15x clone, speed in [0.06,0.096]. Compare directly against this run's OWN 2M reading (0.727x/1.27x) and longrun13's paired result before drawing a lineage-wide conclusion.

**verdict**: FAIL-as-budget-scaling: 4M steps (log-std-anneal-frac 0.3, 2.8M steps at converged std=0.041) did NOT recover seed17 toward pd9. DR-0 det progress 0.719x clone (need >=0.9x), slip 1.32x clone (cap 1.15x), speed 0.058 m/s (floor 0.06) -- flat/slightly worse than this seeds own 2M reading (0.727x/1.27x). Own-DR matches (0.55x prog raw, slip worse under sto). Zero falls, gait_valid 6/6 all 24 eps, six legs cycling clean video, no drag/flag-leg. W-and-B ep_rew quarters [-245,-379,104,193]: collapsed then partially recovered, mirroring -9-cont1s pattern, but recovery did not translate into gate-metric movement. Confirms the pre-registered prediction-if-false: real seed-dependent basin selection at this budget, independent of anneal timing.

