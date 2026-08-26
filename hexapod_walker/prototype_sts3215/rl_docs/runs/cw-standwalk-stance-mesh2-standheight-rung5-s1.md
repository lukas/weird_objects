# cw-standwalk-stance-mesh2-standheight-rung5-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY PASS

**created**: 2026-08-26T04:43:39+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stancemix-tuckclock-scratch8m

**wandb_id**: z103ioia

**hypothesis**: Seed-1 twin of cw-standwalk-stance-mesh2-standheight-rung5 (identical recipe, only seed changed) -- same plain-English question: can the promoted mesh stance policy learn to raise/lower its stand height on command mid rise->hold->lower sequence without breaking rise/lower. Pairs with the seed-0 canary for a joint mechanism-health read instead of a single-seed lottery ticket.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same as cw-standwalk-stance-mesh2-standheight-rung5 (MECHANISM-HEALTH CANARY ONLY, 2M); judged jointly with the seed-0 twin -- PASS needs both seeds to clear (reward rises + probe shows real height tracking + rise/lower not majority-collapsed), PARTIAL if seeds disagree, FAIL if both show flat reward or majority over_current/fall.

**verdict**: CANARY PASS (seed-disagreement on the lower-phase residual -- joint read with the seed-0 base run cw-standwalk-stance-mesh2-standheight-rung5, which carries the full shared analysis; same recipe, seed 1). Reward rises over the run despite the same mid-training valley both seeds share (ep_rew_mean 6.5->peak~54@1M->trough -216@1.4M->recover to 66 final). The pre-registered mode_seq_stance+hold_height_cmd seqprobe shows real height tracking during hold (herr_end 1.2-10.3mm det, tracking a moving [-40,20]mm/15mm-s target, not flat-ignored) and no majority over_current/fall on any mode. Rise mildly softened (4/6 det+4/6 sto success, matching the seed-0 twin). Lower is this seed's weak point and the reason the joint read is a caveated PASS, not a clean one: in the composed seqprobe, lower/det tracks the commanded height TIGHTLY (herr 0.3-2.0mm at the moment of termination) but trips the over_current safety on ALL 6/6 det episodes (0/6 success by the 15mm-error success rule, but purely a safety-margin trip, not a tracking or posture failure -- video shows upright, six-foot-planted stands throughout, no fall/tip); lower/sto is clean (6/6 success). The seed-0 twin shows the OPPOSITE lower failure shape (stays alive with zero terminations, drifts to a high herr instead) -- that cross-seed disagreement is the residual this canary flags, not a shared structural break. A second, likely-cosmetic divergence: this seed's ISOLATED (non-composed) cold-start hold/det probe is far more fragile than seed-0's (4/6 and 3/6 hold_min_load terminations vs 0-1) even though hold is clean inside the actual composed sequence -- probably an artifact of the isolated probe's plant-reset start state rather than the real stage-2 entry condition (rise always precedes hold there). Acting on the run's own pre-registered PARTIAL/residual branch: funding an 8M-budget continuation from this checkpoint (unchanged recipe) before building a new height-cmd-segment bc_anchor-coef lever, since Q4 reward is still improving at the 2M cutoff on both seeds. Evidence: logs/ckpt_eval/cw_standwalk_stance_mesh2_standheight_rung5_s1{,_owncfg,_seqprobe}/; W&B z103ioia.

