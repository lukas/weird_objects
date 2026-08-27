# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor12-walkretain-base-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY FAIL - MECHANISM

**created**: 2026-08-27T14:12:31+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2-s1

**wandb_id**: q6v6no59

**hypothesis**: Plain English: seed-1 twin of anchor12-walkretain-base -- the walk-retention BC anchor (phase_lock=1, knee_abs=1, teacher dialect; operator design correction 2026-08-27) added to the walk-clean anchor2-s1 baseline, isolating the retention term's own effect on a healthy walk on the second seed. Same predictions as the seed0 twin: walk stays gait_valid 6/6 at or above anchor2-s1's own prog band if the anchor is neutral-or-helpful; regression beyond eval noise means the raw scripted-tripod pull at global coef 3.0 harms a mesh-adapted gait and a per-mode anchor coefficient is the next knob.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY at 2M, joint call with seed0 twin; matched control = anchor2-s1 gate/owncfg evals. WIRING CHECK: train/bc_anchor_fill_walk + bc_anchor_loss_walk nonzero. Joint PASS/FAIL branches identical to the seed0 twin's gate text. Read reward trend per 08-21 first.

**verdict**: CANARY FAIL - MECHANISM (seed1, own scope; JOINT CLOSE alongside the already-verdicted seed0 control, same direction/magnitude). vs matched parent anchor2-s1's own DR-0 gate (byte-identical recipe minus the walk-retention anchor): walk det prog_ratio 0.32->0.16 (-50%), slip/m 4.07->6.47 (+59%); walk sto prog 0.03->0.02, slip 23.90->29.12 (+22%); gait_valid held 6/6 both runs on all walk variants (no anchor4-class freeze introduced); hold/sto term 6/6 both (anchor2-s1's own pre-existing hold_min_load problem, unaffected either way -- the walk anchor doesn't touch hold). Video (contact sheet, walk_det) shows normal six-leg cycling, no catastrophe -- agrees with the metrics: a real quality tax, not a freeze, same shape as seed0's control read (prog -66%/slip +128%) and as anchor11-walkretain-s1's real-catastrophe-rescue read (which paid the identical style of tax while gaining gait_valid). Per this arm's own pre-registered FAIL bar ('either seed') this was already decided on seed0 alone; this seed1 read confirms it cross-seed, closing the joint call cleanly (no divergence). No new lever from this read -- already answered by the running/finishing anchor13/14 per-mode-coefficient wave (train.bc_anchor_walk_coef=1.0 vs the shared 3.0).

