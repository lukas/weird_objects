# cw-standwalk-stance-mesh2-stage2-dualbc1-modeseq1-s1r

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-26T12:15:47+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-modeseq1-s1

**wandb_id**: gmtsnhem

**hypothesis**: Retry of an infra-hung run: does the modeseq1 stage-2 recipe hold up on a second seed? This is a byte-identical relaunch (seed 1) of cw-standwalk-stance-mesh2-stage2-dualbc1-modeseq1-s1, which froze in a sporadic eval/video deadlock at its 1M-step eval boundary (no science signal; seed 0 finished normally). Prediction-if-true: run completes 2M steps with healthy canary probes like seed 0. Prediction-if-false: a second hang at the same boundary = reproducible infra bug, escalate instead of retrying again. Strongest alternative: the hang was seed-correlated compute load, not a race.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same as -modeseq1 seed 0; joint call reads both seeds together.

**verdict**: CANARY FAIL - MECHANISM (own scope, seed 1; joint call pends the seed-0 twin, owned by a concurrent cycle). Plain English: bare PPO fine-tuning DESTROYED all three skills the dual-teacher BC clone started with, instead of holding or improving them. The infra half of this retry's hypothesis is CONFIRMED-TRUE: the byte-identical relaunch sailed past the 1M eval boundary where -s1 froze and completed its full 2M — the -s1 hang was a sporadic eval/video deadlock, not reproducible. The science half fires EVERY pre-registered FAIL clause on the DR-0 own-cfg gate panel (n=6/mode det+sto, strips watched): walk/det creeps 0.44m in 30s (prog 0.19; commanded 0.08 m/s, actual ~0.015) with visually zero translation — park/freeze, the exact pathology this dual-core architecture was built to prevent; walk/sto is a frozen in-place stance (prog med 0.02, slip med 28.25, 2/6 over_current, gait invalid 2/6 with sacrificed legs); hold/det collapses 6/6 hold_min_load (video: progressive splay-to-floor from standing); rise det 5/6 + sto 6/6 over_current-pinned; lower det 5/6 + sto 4/6 over_current. Training curve tells the same story: reward +61 at 400k (BC init still intact) -> collapse to -771 at 1.05M (termination storms: hold_min_load 101/window, over_current 90/window) -> partial-recovery plateau ~-185 by 2M, with every end-of-run canary probe at 0 vs the BC baseline's mixed passes. This is NOT the 08-21 continue case: reward plateaued far BELOW the BC init's own starting behavior and eval agrees with the reward story — the fine-tune erased the BC skills (classic fresh-critic PPO erosion; mode-gated dual-core routing protects cores from OTHER modes' gradients, not from their own mode's fresh-advantage updates). Hardware-ready: no. Next: the pre-registered fallback — train.bc_anchor (stance-only, walk-off) per the arch-gru-dual1 precedent — should be the joint call's route; seed-0's cached W&B end-state shows the same signature (ep_rew -197, canaries 0, track/unload survived 0), but its harness verdict belongs to the concurrent cycle. Own-DR (0.5) owncfg pass was still copying back at verdict time; DR-0 (the easier setting) already fails every clause, so it cannot change the call. Evidence: logs/ckpt_eval/cw_standwalk_stance_mesh2_stage2_dualbc1_modeseq1_s1r_gate/ (report.json + strips), cached W&B gmtsnhem history.

