# DOWNLOAD ANSWER — "if the robot were fixed tomorrow morning, what exactly would we download?"

Created 2026-08-17 (SIM SPRINT, operator ruling ~18:05 UTC). This is
the sprint's living deliverable: the named checkpoints + gate
evidence we would put on the robot the moment it is back. Update it
whenever a verdict changes the answer; facts must agree with
CURRENT_TRUTHS.md and rl_docs/SKILLS.md.

## The answer (2026-08-17): the HIERARCHICAL SESSION COMPOSITION

The measured PRODUCT BASELINE (operator ruling
fb_20260814T205137_33f21c; bulk gate below). Download BOTH
checkpoints + run them under the session controller — this is
genuinely closer than any single policy (both single-model distills
are CI-separated WORSE: sto 0.653/0.697 vs 0.853).

| Piece | Artifact | Integrity |
|---|---|---|
| Stance (rise / hold / lower) | `ppo_goal_cw_stand_footlow2_hard1` (W&B artifact `ckpt-...`, controller cache `rl_move/sim/policies/ppo_goal_cw_stand_footlow2_hard1.zip`) | md5 `680e8e0ef81ea6bfabd5396c2a494e59` |
| Tall walk (joystick drive, own-cfg `goal.walk_obs_body_vel=2`, vel:=ref) | `ppo_goal_cw_dep_bcgait1_hard1` (cache `rl_move/sim/policies/ppo_goal_cw_dep_bcgait1_hard1.zip`) | md5 `b55d54f5059439a590e33349c081d237` |
| Session controller (deploy runner) | explicit grammar with per-mode re-anchor; **entry-slew ON** (`1.5,0.25`); **STOP routes to stance hold** (mandatory — walker creeps ~0.04 m/s at zero command, 0/2773 segments settle); rot60 wrapper default-ON for the full command circle | code on main |

Trained goal profiles ship inside the weights meta (runner contract
test-locked, `rl_move/tests/test_stand_runner.py`).

## Gate evidence

- **Bulk held-out session gate, Cohort c1 (n=600 fresh sessions,
  pre-registered `rl_docs/tracks/hw/SESSION_BULK_GATE.md`):** det
  complete-session zero-fall 290/300 = **0.967** CI [0.940, 0.982],
  every segment type ≥0.983, every cold-start stratum (flat/bridge/
  crouch) ≥0.95, gait_valid 590/590; sto **0.853**. Visual stats:
  slip/m med 1.75, drive height 135 mm. Walking itself: **zero falls
  in all 1,104 drive segments**.
- **Stance gate (footlow2_hard1):** all four pre-registered clauses —
  cold det rises 12/12 incl. targeted all-flat probe (h_err
  0.5–3.4 mm, roll_tail 0.0°), park-free six-foot hold (duty
  0.95–0.99), lower 12/12 det+sto feet-flush, `eval_session` hard
  gates PASS with rise to full 148 mm under the interactive ramp
  (deployed holdbc1_hard1 stalls at 55 mm). Seed-robust
  (`-s1` twin reproduces all clauses).
- **Walk gate (bcgait1_hard1):** in-band walking height (broke the
  crouch-splay wall), positive leg-yaw margin, det slip/m 1.30–1.46,
  sto 1.31–1.51, gait_valid 6/6 all slices, zero falls; retention
  panel friction 0.4–1.6× and 5° floor tilt both PASS; takeoff
  push-probe no worse than deployed tip1.

## Known gaps shipping as-is (the sprint's named targets)

1. **Post-lower rise** — the single weak boundary: sto 0.801
   (over_current-dominated; 100% of the det session failures). Four
   training attempts (postlower1–4) all closed WORSE-or-short of the
   parent; best (c4) det 0.872 / sto 0.690 vs parent 0.967 / 0.801.
   The download today ships the PARENT (best measured). Next lever is
   the open `[operator]` fork (STATUS.md WAITING-ON): (a) remaining-
   rise runner semantics (train==deploy) vs (b) price post-lower rise
   in reward.
2. **Takeoff roll transient** — a hardware phenomenon sim survives
   (SIM.md gap 4); the composed entry-slew ramp is the best measured
   mitigation (saved 5/9 falls in the 144-rollout calibrated proxy,
   caused none). Bench reps are `[operator]`, parked during repair.
3. **Learned stand-up on hardware unproven** — pre-repair bench:
   learned stand tripped tilt_roll 10/10 (deployed holdbc1_hard1;
   footlow2_hard1 never bench-tested). The scripted stand glide stays
   the hardware fallback until the new stance is bench-promoted.

## Fallbacks already on the robot

Deployed pair `holdbc1_hard1` (stance; fails the session gate) +
`dep-tip1` (walk); scripted tripod gait + scripted stand/sit glides.
The download above supersedes both on every sim gate; promotion
itself remains an operator bench call.
