# cw-arch-hist16-r7-c4

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T22:11:22+00:00

**pod**: hexapod-mjx-train-3

**steps**: 40000000

**parent**: cw-arch-hist16-r7-c3

**wandb_id**: d6rd9a1r

**hardware_ready**: False

**hypothesis**: Plain English: keep training the 16-frame architecture walker to see if the slip/economy gap to the deployment champion keeps closing or has plateaued. cw-arch-hist16-r7-c3's +40M continuation PASSED and was still improving (det slip/m 1.16, sto 1.27, both closer to the champion band 0.89-1.13 than c1's 1.14/1.33) with zero stability regression and a clean joystick gate. One variable vs c3: +40M more steps, same recipe, nothing else changed. If-true: slip/m moves fully inside the champion band and sto keeps closing -- the architecture line closes the gap by exposure alone, no reward change ever needed. If-false: slip/m plateaus outside the band despite the extra steps -- the gap is structural (contact-pricing/architecture), not a training-budget question, and the line should try a different lever before another blind continuation.

**gate**: own-cfg DR0.5 det+sto 6/6 gv, 0 term, prog med >=1.0; slip/m det improves or holds vs c3's 1.13 (not regressing), sto improves or holds vs c3's 1.31; DR0 gate det+sto 6/6 gv, 0 term; JOYSTICK GATE (eval_drive DR0.2) 0 falls; video clean six-leg cycling, no flag leg

**verdict**: PASS (gate met) but the hypothesis's if-false branch is what happened: the slip/economy gap did NOT keep closing with +40M more steps, it plateaued. Own-cfg DR0.5 det slip med 1.15 vs c3's 1.13 (flat, +2%, within noise), sto 1.28 vs c3's 1.31 (flat, -2%) -- essentially no net movement after 3 prior continuations of steady improvement (c1->c3). DR0 gate is more telling: det 1.02 (now clearly inside the champion band 0.89-1.13) but sto 1.42, WORSE than c3's 1.27 by +12%, outside noise. gv 6/6 all 4 passes, 0 term, prog med >=1.0 everywhere. JOYSTICK GATE (eval_drive DR0.2, ran myself): PASS, 0 falls across full direction+flip-stress panel. Video (det+sto, all 6 eps each pass): clean six-leg swing/stance cycling throughout, no flag-leg/drag/skate -- same ordinary gait as c1-c3, no new pathology. Architecture-line only (joint_walk task, not the dep-line's deployment-exact obs contract) -- not itself a hardware candidate. DECISION: closes the 'does exposure alone close the gap' question for the r7 line -- it does not; per RESEARCH_RULES two-flat-continuations rule, stop blind step-budget continuations here (c3 pre-registered this as the last one before judging). Remaining gap is CURRENT_TRUTHS open problem 1 (contact/current pricing, k_current=0, needs a load-dependent holding-current fit), not architecture depth. No c5 respec.

