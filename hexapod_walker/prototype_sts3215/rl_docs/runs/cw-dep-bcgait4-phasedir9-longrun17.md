# cw-dep-bcgait4-phasedir9-longrun17

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-22T13:46:44+00:00

**pod**: hexapod-mjx-train-0

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-seed17

**hardware_ready**: False

**hypothesis**: Companion arm to longrun13, same budget-vs-mechanism question on the seed that did NOT near-pass at 2M: seed17 landed at/below pd8's level (0.727x progress, 1.27x slip) with reward still declining at cutoff, not obviously converged. Identical stack+seed(17) as phasedir9-seed17, ONLY --steps 2M->4M and --log-std-anneal-frac 0.6->0.3 (anneal still ends at the same absolute ~1.2M step; policy gets 2.8M steps at converged std=0.041 instead of ~800k). Prediction-if-true: seed17 recovers toward or past pd9's 2M numbers with the extra converged-regime budget -- seed17's bad 2M reading was itself undertrained, not a worse basin. Prediction-if-false: seed17 stays flat/worse through 4M despite the same schedule that (maybe) helps seed13 -- real seed-dependent basin selection at this budget, independent of anneal timing, which redirects investment toward the phase-lock/anchor dig-in and away from budget scaling for either seed.

**gate**: Same clone-relative forward panel as pd9 (logs/ckpt_eval/phasedir3_clone_control_gate, DR-0 det+sto). Report progress/slip/speed/falls at final (4M) checkpoint plus the ~2M mid-run eval for trend. PASS = zero falls, gait_valid 6/6, progress >=0.9x clone, slip <=1.15x clone, speed in [0.06,0.096]. Compare directly against this run's OWN 2M reading (0.727x/1.27x) and longrun13's paired result before drawing a lineage-wide conclusion.

**verdict**: FAIL (flat; confirms the pre-registered prediction-if-false): +2M steps beyond phasedir9-seed17's own budget (2M->4M, same anneal end-step ~1.2M) left det-gate progress statistically FLAT (0.727x clone -> 0.714x, 0.55m/0.77m) and slip flat-to-worse (1.27x -> 1.323x, 2.50/1.89) versus this run's own 2M reading, despite W&B ep_rew_mean also ending strongly positive/rising (-378 -> +104 -> +193). Zero falls, gait_valid 6/6 det+sto, clean video. Exactly matches the pre-registered prediction-if-false ('seed17 stays flat/worse through 4M despite the same schedule') -- seed17's bad 2M reading was NOT undertraining, it is a real seed-dependent outcome independent of anneal timing. Paired with longrun13 (same cycle, WORSE not just flat), this closes the budget question for this reward stack: reward keeps rising post-anneal but the clone-relative gate does not track it -- a reward<->eval misalignment per the 08-21 ruling, not an undertrained-run false negative. Hands off cleanly to the BC-anchor/phase-lock family-boundary dig-in.

