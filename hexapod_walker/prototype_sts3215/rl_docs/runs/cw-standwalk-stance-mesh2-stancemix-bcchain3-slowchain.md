# cw-standwalk-stance-mesh2-stancemix-bcchain3-slowchain

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T16:04:27+00:00

**pod**: hexapod-mjx-train-0

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-stancemix-bcchain3-stdanneal

**wandb_id**: kza9ep2s

**hypothesis**: Does the rise pace-halving lever validated on the ISOLATED rise lineage (riseonly-bcchain3-slowchain: bc_anchor_lookahead_s 0.5->0.25s, min_h_ahead_mm 15->8mm, which cut deep-start cur_p95 2.64A->1.85A and raised DR-0 det valid_plant 2/6->3/6) ALSO unpin rise's deep-start press-up when ported into the FULL hold+rise+lower mix, warm-started from the just-PASSED stancemix-bcchain3-stdanneal checkpoint (hold 6/6+6/6 zero-term @0.78A, lower 6/6+6/6 @<=1.4mm, rise det 2/6 with deep starts pinned 2.64A) -- without disturbing the hold/lower parity that checkpoint just achieved? Warm-start (not from-scratch) so hold/lower keep their solved weights; log-std pinned at -4 (no re-anneal, mirroring the isolated cont8 continuation) since noise re-injection was already shown NOT to be the rise blocker. Prediction-if-true: rise DR-0 det valid_plant rises above 2/6 and/or deep-start cur_p95 median falls below 2.64A toward slowchain's 1.85A, while hold stays >=5/6+5/6 zero-term <=1.0A and lower stays >=4/6 honest (<=10mm) -- pace transfers cleanly into the mix. Prediction-if-false: rise stays pinned at 2.64A/2-6 valid_plant despite the pace change (mix dynamics differ from isolated), or hold/lower regress (the mix trades one mode's gain for another's loss under shared policy capacity) -- either result means the pace fix must be re-derived in-mix, not simply ported.

**gate**: 8M continuation (same budget class), DR-0 + own-DR(0.2) det+sto n=6+6 per mode. PASS: rise DR-0 det valid_plant >=3/6 (beats stdanneal's 2/6) AND deep-start cur_p95 median <2.64A AND hold det+sto stay >=5/6 valid_plant with zero hold_min_load/over_current terms (cur_p95<=1.0A) AND lower det stays >=4/6 honest descents (height_err_end<=10mm) -- pace lever transfers into the mix without cross-mode cost. PARTIAL: rise improves (valid_plant count up and/or cur_p95 median falls) without crossing the full bar, hold/lower unchanged or only mildly softer -- pace helps but doesn't finish the job in-mix either. FAIL: rise unchanged (still <=2/6, cur_p95 still ~2.64A) OR any of hold/lower regresses vs the stdanneal parent (loses its just-achieved zero-term/clean-descent parity) -- the isolated fix does not transfer, the mix needs its own lever.

**verdict**: FAIL per pre-registered bar: the half-pace chain lever does NOT fix rise inside the 3-mode mix, though hold/lower transfer perfectly. DR-0: hold det+sto 12/12 valid_plant zero terms (cur_p95 0.73-0.87A <=1.0A bar); lower det+sto 12/12 honest full descents (herr_end 0.1-0.8mm, clean rolls, <=0.94A); rise det 1/6 (crouch only) + sto 1/6, ALL 9 deep starts (flat/bridge/rsi) pinned at the 2.64A over_current ceiling, herr_end 12-81mm — same as stdanneal parent's <=2/6, so gate FAIL clause fires (isolated fix does not transfer). Own-DR(0.2): hold 11/12, lower 12/12, rise 0/12. KEY DIAGNOSTIC strengthening rung-9: in-mix bc_anchor_loss_rise fell to 0.027-0.035, BELOW the ~0.05 plateau that capped every riseonly arm, and env/rise_score hit 0.56 (riseonly peak ~0.43), yet SCORE/raise_success stayed 0.0 for all 8M steps — the policy tracks the borrowed primitive-family reference better than ever and still cannot press up from deep starts without pinning current. Tracking capacity is NOT the bottleneck; the reference's flat-segment posture is torque-infeasible on the 3.5kg mesh model. This is exactly what the in-flight rung-9 meshref arms (other cycle) attack. Next: no further mix funding until a rise recipe passes in isolation; re-run this exact mix once meshref (or successor) produces a rise with det>=4/6. Evidence: logs/ckpt_eval/cw_standwalk_stance_mesh2_stancemix_bcchain3_slowchain_{gate,owncfg}/, W&B kza9ep2s.

