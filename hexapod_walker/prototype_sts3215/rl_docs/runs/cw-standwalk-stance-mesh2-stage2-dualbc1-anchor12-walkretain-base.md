# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor12-walkretain-base

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY FAIL - MECHANISM

**created**: 2026-08-27T14:08:10+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2

**wandb_id**: y8vpddca

**hypothesis**: Plain English: CONTROL arm for the walk-retention anchor (operator design correction 2026-08-27) -- add the same walk-tick BC anchor (phase_lock=1, knee_abs=1, walk teacher stotight45-seed13's own anchor dialect) to the walk-CLEAN anchor2 baseline (no log_std split/anneal), isolating what the retention term itself does to a healthy-but-weak walk. anchor2's walk is gait_valid 6/6 with prog_ratio 0.32-0.38 det DR-0 (a weak crawl); the anchor supplies a which-way-to-move action gradient the walk core currently never gets in this lineage. Prediction-if-true: walk stays gait_valid 6/6 with prog at or above the anchor2 band (the anchor can only help or be neutral on a clean gait) and stance reads stay in anchor2's own band (stance anchor + isolate_update unchanged). Prediction-if-false: walk prog/slip regress beyond eval noise vs anchor2's matched evals -- the raw scripted-tripod target at global coef 3.0 (3x the teacher's own walk dose of 1.0) over-constrains or mismatches the mesh-adapted gait; next knob is a per-mode anchor coefficient (code, default-off), and the anchor11 rescue read must be reinterpreted with that pull in mind. Strongest alternative: no detectable difference at 2M (anchor income too small vs PPO gradients at this coef) -- then the retention term needs a dose read, not abandonment.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY at 2M, joint call with -s1 twin; matched controls = anchor2{,-s1} gate/owncfg evals. WIRING CHECK: train/bc_anchor_fill_walk + bc_anchor_loss_walk nonzero across rollouts. PASS if on BOTH seeds walk det DR-0 gait_valid 6/6 with prog_ratio >= anchor2's 0.32-0.38 band (within eval noise) and slip not worse beyond noise, AND hold/lower/rise reads stay in anchor2's band. FAIL if walk prog/slip/gait_valid regress beyond eval noise on either seed (anchor pull harms a healthy gait -> per-mode anchor coef is the next knob before any wider adoption of the retention term). This control decides whether anchor11's PASS (if any) can be attributed to retention rather than to the anchor replacing the gait. Read reward trend per 08-21 first.

**verdict**: CANARY FAIL - MECHANISM (seed0, decidable alone per this arm's own pre-registered FAIL bar 'on either seed'): turning ON train.bc_anchor_walk=1 (bc_anchor_coef=3.0, phase_lock+knee_abs) on the walk-CLEAN anchor2 recipe measurably harms the walk it was meant to protect. DR-0 gate vs anchor2's own DR-0 gate (matched parent, same seed0): det walk prog_ratio 0.38->0.13 (-66%), slip/m 3.63->8.27 (+128%); sto lower 5/6->1/6 (collapsed); sto hold stayed 0/6 (already bad). gait_valid held 6/6 det+sto both runs (no anchor4-class catastrophe) and sto walk actually improved from 5/6-sacrificed-legs to 6/6-clean -- so this is not a freeze/shuffle catastrophe, it is the anchor's PULL taxing an already-working gait, exactly the -2.35(b) failure mode. own-DR (dr0.5) confirms the same direction (prog 0.37->0.19, slip 3.56->6.22). -s1 (seed1) is still training under this cycle's non-ownership list; this FAIL stands on seed0 alone per the gate's explicit either-seed wording and should be read jointly with -s1 when it lands. NOT HARDWARE-READY. Built+unit-tested (9 new tests, all green) the prescribed follow-up train.bc_anchor_walk_coef (default -1 unset = bit-exact legacy single-coef branch; when set, splits each aux minibatch by bc_mode into a walk group and a stance group, each scaled by its own coefficient).

