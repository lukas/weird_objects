# cw-omni-trans1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-11T02:02:14+00:00

**pod**: hexapod-mjx-train-9

**steps**: 40000000

**parent**: cw-arch-hist16-dep1

**wandb_id**: lcg1wfpu

**hardware_ready**: False

**hypothesis**: TRANSLATION-ONLY omni arm (operator re-scope, 08-10 late: the robot has no camera hence no meaningful front — it does not NEED to turn; every reward ingredient behind the mirror1-r1 freeze exploit lived in the turn machinery). This arm asks the simpler question the deliverable actually requires: can the proven dep1 recipe learn to walk in EVERY commanded direction when trained on full-circle headings, with turning deleted from the task? Three deltas from the triaged-PASS cw-arch-hist16-dep1, nothing else: goal.walk_heading_max_rad=3.14159 (uniform +-180deg incl backward — dep1 measured WORSE than frozen on backward, 0.069 trk_err), reward.k_current=0 (hardware ruling: walking measured cheaper than standing), train.mirror_loss_coef=1.0 (left-right regularizer, proven healthy in the mirror line; symmetric headings should not have chiral gaits). No yaw command, no k_yaw_still tax, no turn-in-place episodes — the park attractor's income sources are structurally absent. If-true (full direction panel tracks near dep1's forward-cone quality ~0.03): the drivable translation policy exists; turning becomes a separate later skill, and the rot-60 exact-equivariance lever stays in reserve. If-false (backward/lateral stay near-frozen while forward matches dep1): full-circle exposure alone cannot buy new directions in 40M — next lever is rot-60 equivariance (exact 6-fold symmetry: backward IS forward with legs relabeled), not reward surgery.

**gate**: At 40M: eval_drive full panel at --heading-max-deg 180 (fwd/back/left/right/diags all GATE scenarios): 0 falls AND trk_err <= 0.035 on every direction (dep1's own forward band 0.029-0.031; frozen = 0.05); gait_valid 6/6 det at own DR and DR0; video shows ordinary swing/stance in backward and lateral clips (no moonwalk-drag, no spin-and-walk-forward cheat); mirror_sym_loss low throughout; kill-early on the r1 signature (det forward collapse with frozen episodes out-earning walking, or std past 2x start).

**verdict**: FAIL -- known WALK exploit: paddle/march-in-place collapse, not ordinary gait. Legs 1 and 4 stay planted ~90-99% duty the whole episode while the other four churn tiny ~0.01m strides; slip_per_m 3-13 (vs champion ~1.2-1.5); progress_ratio med 0.51 det / 0.22 sto, 0/6 success any mode/DR; trk_err grossly misses the <=0.035 gate. train/std climbed continuously 0.37->1.38 (3.7x start), blowing past the pre-registered 2x kill-early signature with no plateau. Mirror-symmetry remains statistically untested -- this is a third distinct degenerate omni pathology (mirror1 freeze, mirror2/dr02 leg-sacrifice, trans1 paddle-stall) even with the yaw stack fully removed.

