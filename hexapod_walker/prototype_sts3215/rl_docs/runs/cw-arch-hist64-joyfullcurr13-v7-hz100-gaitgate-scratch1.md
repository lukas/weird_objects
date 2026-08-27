# cw-arch-hist64-joyfullcurr13-v7-hz100-gaitgate-scratch1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-25T02:18:37+00:00

**pod**: hexapod-mjx-train-9

**steps**: 40000000

**parent**: cw-arch-hist64-joyfullcurr13-v7-hz100-scratch-s0-r1

**hypothesis**: Plain English: if the proven anti-leg-sacrifice reward gate (reward.walk_gait_gate) is baked in FROM STEP 0 instead of added after the fact, does the leg-sacrifice fingerprint ({0,2,5}/{3,5}/{0,2,3,5}, confirmed on 3 independent lineages so far) ever form at all? This is the decisive from-scratch sibling to the cont1 continuation diagnostic (same lever, applied at the start instead of after 40M steps of convergence into the bad optimum) -- walkcurr's own repeated finding is that reward-side fixes on an ALREADY-parked policy usually fail to dislodge it even when the fix is aligned, so a from-scratch test is needed to separate 'the gate prevents the pathology' from 'the gate can't undo a converged basin'. Mechanism: walk_gait_gate multiplicatively collapses walk/progress/tracking income toward the (1-g) floor whenever ANY support leg has not completed a real swing within a 2s trailing window (MIN across legs, not mean, so no subset of legs can be sacrificed to fund the others) -- bank-proven this cycle for exactly the 'sacrifice any subset' cheat class (test_walk_gait_gate_collapses_flag_leg_income, recalibrated threshold; test_walk_gait_gate_keeps_honest_gait_income; quadwalk mid-pin sibling) but was absent from every V7/100Hz joyfullcurr13 recipe run so far. Byte-identical recipe to the FAIL-verdicted scratch-s0-r1 parent (same seed 0, same V7/100Hz/hist64-MLP stack, same 40M budget) except this one lever. Prediction-if-true: frontier promotes past b0, held-out gait_valid clears real fractions (not 0/6), no leg permanently sub-0.1-duty. Prediction-if-false: reward stays flat/depressed relative to the parent's own trajectory with the frontier stuck at b0 (the gate makes sacrifice unprofitable but the policy can't find ANY six-leg-cycling solution at this budget/architecture -- a genuine capability ceiling, not a pricing gap) -- or a NEW degenerate escape appears (e.g. all six legs barely-qualifying shuffle) that the gate's threshold doesn't catch, which would be a mechanism-hardening finding in its own right. Strongest alternative: leg sacrifice is a genuine local-optimum-selection accident of tripod-style gait learning on a bilaterally symmetric platform, independent of this specific pricing gap, and the gate just makes THIS particular escape route more expensive without addressing why some subset gets abandoned in the first place.

**gate**: PASS: frontier promotes past b0 with reward/eval agreement, held-out joygate/owncfg gait_valid clears a real fraction (not the parent's 0/6), no single leg pinned <0.1 or >0.9 duty across the panel. PARTIAL: genuinely learning (reward trajectory clears the from-scratch valley, frontier moves) but still short of full gait_valid clearance by 40M -- continuation candidate per 08-21. FAIL: reward stays in/near the valley with frontier pinned at b0 through 40M (mirroring the parent's own shape) AND gait_valid stays 0 -- a genuine capability ceiling at this architecture/budget, escalate past reward-mechanism levers entirely (architecture/curriculum, not pricing).

**refused_reason**: discovery runs cap at 2000000 steps (asked 40000000): the question is 'did qualitatively correct behavior emerge?' - continue as --phase hardening with --evidence.

