# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor11-walkretain-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY PASS - MECHANISM (PARTIAL) - JOINT CLOSE

**created**: 2026-08-27T14:14:42+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor6b-logstdsplit-fix-s1

**wandb_id**: taxhmfsw

**hypothesis**: Plain English: seed-1 twin of anchor11-walkretain -- make walking regression hurt during training (operator design correction 2026-08-27): turn the walk-tick BC anchor ON (phase_lock=1, knee_abs=1, the walk teacher stotight45-seed13's own anchor dialect) on the anchor6b-logstdsplit-fix-s1 recipe, THE known catastrophe seed (walk 0/6 gait_valid total collapse in the parent). This is the direct rescue test: if an in-loss walk-retention term prevents the collapse on the seed where every architecture-side fix (log_std split, detach_trunk) failed, the operator's reward/loss-guardrail thesis is behaviorally confirmed. Prediction-if-true: walk det DR-0 gait_valid recovers to >=5/6, prog_ratio>=0.2, zero sacrificed legs, while hold keeps the stance-core anneal gain. Prediction-if-false: collapse recurs with the anchor genuinely wired (check train/bc_anchor_loss_walk + bc_anchor_fill_walk nonzero every rollout FIRST -- a silently-empty walk buffer is an INFRA read, not a mechanism read); then route to per-mode-group objective normalization, not another architecture arm.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY at 2M, joint call with seed0 twin; matched control = anchor6b-logstdsplit-fix-s1 gate/owncfg evals (walk 0/6 gait_valid catastrophe). WIRING CHECK FIRST: train/bc_anchor_fill_walk and bc_anchor_loss_walk must be nonzero across rollouts before any mechanism verdict. PASS/PARTIAL/FAIL branches identical to seed0 twin's gate text (joint rule): PASS = both seeds walk gait_valid>=5/6 + prog>=0.2 + 0 sacrificed legs AND hold/sto DR-0 term <=2/6. FAIL on this seed = walk still anchor4-class collapsed despite a live walk anchor -- scripted-anchor retention refuted for the catastrophe class, route to per-mode objective normalization build. Read reward trend per 08-21 first.

**verdict**: CANARY PASS - MECHANISM (PARTIAL), JOINT CLOSE with already-verdicted anchor11-walkretain (seed0, own-scope CANARY FAIL - MECHANISM). Wiring check first: train/bc_anchor_{fill,loss}_walk nonzero all 24 rollouts. Seed1 IS the real anchor6b-logstdsplit-fix-s1 catastrophe target (parent: walk det DR-0 gait_valid 0/6, sac legs [0,1,5]/[0,1,2,4,5], prog_ratio 0.01, slip/m 25.96 -- a static near-freeze). With the walk-retention anchor: walk det DR-0 gait_valid 6/6, ZERO sacrificed legs, prog_ratio 0.16, slip/m 6.69 -- a genuine rescue from total collapse to a clean six-leg gait (video-confirmed: normal tripod-ish stance/swing cycling, no crouch/freeze). hold/sto term 0/6 (anneal gain intact). Per this arm's own pre-registered branches: not FAIL (no anchor4-class collapse on either seed -- the opposite happened here), not full PASS (prog_ratio 0.16 seed1 / 0.08 seed0, both below the 0.2 bar), closest to PARTIAL-A's spirit (walk protected both seeds, hold/sto stayed <=2/6 both) with the added twist that 'protected' here means 'rescued from real collapse' not just 'undamaged.' Net joint read: the in-loss walk-retention anchor DOES prevent/reverse the anchor4-class leg-sacrifice catastrophe on its designated target seed, but the SHARED coef=3.0 dose taxes achieved speed on both seeds (matches anchor12-walkretain-base's control finding on the walk-clean recipe) -- this is exactly the evidence -2.35(b)'s already-launched per-mode-coefficient wave (anchor13/14-walkretaincoef1, running/finished this cycle, results pending) was funded to fix. No new lever opened by this read; it confirms the existing plan.

