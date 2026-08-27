# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor12-walkretain-base-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-27T14:12:31+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2-s1

**wandb_id**: q6v6no59

**hypothesis**: Plain English: seed-1 twin of anchor12-walkretain-base -- the walk-retention BC anchor (phase_lock=1, knee_abs=1, teacher dialect; operator design correction 2026-08-27) added to the walk-clean anchor2-s1 baseline, isolating the retention term's own effect on a healthy walk on the second seed. Same predictions as the seed0 twin: walk stays gait_valid 6/6 at or above anchor2-s1's own prog band if the anchor is neutral-or-helpful; regression beyond eval noise means the raw scripted-tripod pull at global coef 3.0 harms a mesh-adapted gait and a per-mode anchor coefficient is the next knob.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY at 2M, joint call with seed0 twin; matched control = anchor2-s1 gate/owncfg evals. WIRING CHECK: train/bc_anchor_fill_walk + bc_anchor_loss_walk nonzero. Joint PASS/FAIL branches identical to the seed0 twin's gate text. Read reward trend per 08-21 first.

