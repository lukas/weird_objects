# cw-uni-rfix-fresh1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T13:50:03+00:00

**pod**: hexapod-mjx-train-1

**steps**: 18000000

**wandb_id**: 5yzoxq1d

**hardware_ready**: False

**hypothesis**: Init arm of the c69/c71 uni-line DIG-IN: the MISSING from-scratch control. Every rung of the mix ladder (walk=.7/.4/.2/.0) warm-started the SAME walk-only champion (joyjit_dr05, step0 lineage: goal-mix walk=1.0 since fresh init) - init was never varied, and that lineage is measurably blind to the only obs channel that distinguishes rise/lower from a zero-velocity walk (dA/d(height_ref) 0.22x proprio avg = unused-channel noise floor, vs 4.2x for the from-scratch stance champion; no mode one-hot exists in the goal obs). This arm: NO --init-from, from-scratch field standard (log_std 0, ent 0.005, DR 0.2 per cw-dep-fresh1), same walk env / walk=0 hold=.1 rise=.45 lower=.45 mix / 15s episodes / fixed pricing as cw-uni-rfix-warm1 (reward.rise_finish_gate_signed=1 + rise_income_prog_gate=1). Also re-proves rise-from-scratch on the post-273ebde sim (the stance line's rise predates shin-floor collision - flagged suspect near ground). If-true (fresh learns rise/lower where rfix-warm1 stays at 0): warm-start skill acquisition is confirmed as the uni-line blocker - unified deliverable should go distill/two-policy, not fine-tune grafting. If-false (BOTH arms fail): the walk-env rise/lower task itself is implicated (post-fix sim contact or 15s budget), not the init - next probe is the stance-line joint_goal recipe on the fixed sim.

**gate**: own-cfg DR0.2 rise AND lower success >=5/6 det each by 18M; hold quiet (height_err_end<=8mm); VIDEO: no leg-through-floor; no walk retention clause (walk never trained)

**verdict**: FAIL, and the if-true branch is refuted: from-scratch is strictly WORSE than the warm start, not better. OBSERVATIONS: 0/12 posture-strict (rise 0/6, lower 0/6 det); every lower episode is the same fixed tripod -- legs 0/2/4 held 81-186mm up (duty 0.07-0.23) while h_err hits 0.5-3.2mm and banks ~+400/ep; rise episodes cheat the same way plus 3/6 over_current terminations (cur_max 2.5-2.6A); video shows legs pointed skyward, body propped on 3 legs. Training reward climbed steadily 47->201 -- all of it kernel-farming on the unpriced end-posture hole. INTERPRETATION: init is NOT the uni-line blocker (warm1's lower went 6/6 on the same pricing). Both arms exploit the same hole -- height income with no foot-loading requirement -- and the from-scratch policy, unconstrained by a walking prior, collapses fully into it and into over-current territory. The walking prior is protective, not blinding; the measured height-channel 'blindness' (dA/d(height_ref) 0.22x) was overcome by fine-tuning. VERDICT: FAIL; closes the distill/two-policy argument for now (evidence: warm1 lower 6/6 vs fresh1 0/6). The 'walk-env rise task broken' alternative is also narrowed: lower is learnable in this env, rise fails for the named pricing reason in BOTH arms. HYPOTHESIS STATUS: refuted (if-true); the pre-registered both-fail branch is superseded by warm1's partial pass.

