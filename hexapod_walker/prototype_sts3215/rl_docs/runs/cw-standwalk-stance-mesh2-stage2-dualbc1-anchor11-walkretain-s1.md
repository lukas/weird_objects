# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor11-walkretain-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-27T14:14:42+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor6b-logstdsplit-fix-s1

**wandb_id**: taxhmfsw

**hypothesis**: Plain English: seed-1 twin of anchor11-walkretain -- make walking regression hurt during training (operator design correction 2026-08-27): turn the walk-tick BC anchor ON (phase_lock=1, knee_abs=1, the walk teacher stotight45-seed13's own anchor dialect) on the anchor6b-logstdsplit-fix-s1 recipe, THE known catastrophe seed (walk 0/6 gait_valid total collapse in the parent). This is the direct rescue test: if an in-loss walk-retention term prevents the collapse on the seed where every architecture-side fix (log_std split, detach_trunk) failed, the operator's reward/loss-guardrail thesis is behaviorally confirmed. Prediction-if-true: walk det DR-0 gait_valid recovers to >=5/6, prog_ratio>=0.2, zero sacrificed legs, while hold keeps the stance-core anneal gain. Prediction-if-false: collapse recurs with the anchor genuinely wired (check train/bc_anchor_loss_walk + bc_anchor_fill_walk nonzero every rollout FIRST -- a silently-empty walk buffer is an INFRA read, not a mechanism read); then route to per-mode-group objective normalization, not another architecture arm.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY at 2M, joint call with seed0 twin; matched control = anchor6b-logstdsplit-fix-s1 gate/owncfg evals (walk 0/6 gait_valid catastrophe). WIRING CHECK FIRST: train/bc_anchor_fill_walk and bc_anchor_loss_walk must be nonzero across rollouts before any mechanism verdict. PASS/PARTIAL/FAIL branches identical to seed0 twin's gate text (joint rule): PASS = both seeds walk gait_valid>=5/6 + prog>=0.2 + 0 sacrificed legs AND hold/sto DR-0 term <=2/6. FAIL on this seed = walk still anchor4-class collapsed despite a live walk anchor -- scripted-anchor retention refuted for the catastrophe class, route to per-mode objective normalization build. Read reward trend per 08-21 first.

