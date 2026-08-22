# CURRENT TRUTHS - accepted facts and rulings

Last compacted: 2026-08-21 (two-track reset; folded the 08-22
plant-gate/bank findings). Accepted current facts, not narrative. If
history or old prose disagrees with this file, this file wins.

## Mission

Two goals, and only two, until both gates are green (operator,
08-21): (1) `joystick` — RL from the scripted programmatic gait to
joystick control, gated on 60 s of command-following in MuJoCo with
zero falls and teacher-band slip; (2) `amp` — the from-scratch AMP
program in `rl_docs/AMP_LOCOMOTION.md` (no Isaac Lab; MJX stack;
build all tools; done at M5 MuJoCo transfer). Operator-launched
out-of-scope runs get honest triage but no agent follow-ups.

## Current top rulings (operator, 08-21/08-22)

- REWARD/EVAL AGREEMENT FIRST. Every run triage must compare reward
  trend with gate/eval trend before proposing more seeds or budget. If
  eval is unsatisfactory and flat/down while reward is rising, assume a
  reward/eval/simulator mismatch until proven otherwise. Audit/fix the
  objective (including simulator/contact/servo modeling if needed)
  before same-recipe seed sweeps. Reward-flat + eval-flat means stuck
  signal/mechanism, not "try seeds" by default
  (`RUN_INTERPRETATION_RULES.md`).
- Bad evals/canaries with reward and eval both improving can justify a
  continuation. Bad evals with rising reward but non-improving evals
  are MISALIGNED first, not UNDERTRAINED. A known exploit on video
  means misalignment to repair, not a lineage kill.
- No operator pauses: assume-and-go with recorded assumptions. Only
  physical-robot access and spend approvals wait.
- While either gate is unmet, an idle fleet next to an empty backlog
  is the failure state; build tools, fund continuations, queue the
  next milestone arm.
- SIM SPRINT and the seven-track structure are superseded.

## Facts that feed the two tracks

- **JOYSTICK DONE GATE: FIRST MEASURED PASS (08-22 ~17:3x)** —
  `cw-dep-bcgait4-phasedir9-longrun17-stotight45` (longrun17 recipe,
  fresh reinit, single change `--log-std-final` -3.2 -> -4.5, final
  std 0.011) passes the versioned 60s randomized session gate
  (`eval_joystick_gate`, held-out stress_mix, n=48, det+sto, DR-0 +
  own-DR 0.35): pass=true, zero falls 48/48, gait_valid 48/48, no
  sacrificed legs, combined slip 2.671 <= 2.9, dir 38.6 <= 40, every
  mode individually under caps (thinnest: own-DR sto slip 2.859).
  Root cause of the old sto failure CONFIRMED as trained-noise-floor
  slip: sto slip fell monotonically across the -3.6/-4.0/-4.5 dose
  grid; siblings near-miss (dir 40.7 / slip 2.94). Trade recorded:
  det softened vs longrun17 (session det slip 2.30->2.55, 15s rung
  prog 1.02x->0.85x clone) but stays under every cap. Videos watched
  (det+sto, both DR): clean six-leg gait. Checkpoint on controller,
  md5 9fb86d18. Promotion/formal gate-green declaration awaiting
  operator confirmation (q_20260822T1730Z); until then treat
  stotight45 as the track's champion candidate — champions
  append-only.

- The scripted tripod teacher is verified clean at the measured
  tibia-150 plant (commit a4beb8af): 0.06-0.10 m/s x 4 headings, zero
  falls, slip/m 1.4-2.9, full fast servo profile. It is the joystick
  starting point and the amp motion-prior seed.
- The phase-conditioned BC clone
  `ppo_goal_cw_bcgait_init_fullprof_phase1` (holdout act err 0.0040)
  passes the entire direction-first curriculum with ZERO RL: all
  fixed headings incl. rear, irregular heading changes, stops.
- Five fast-gait RL levers failed AS RUN (faster cadence,
  k_walk_cmd_track, speed-obs+charges, +4M steps,
  phase-obs+fixed-speed). Under the 08-21 ruling these are
  reward<->eval alignment failures, not proof RL cannot improve the
  clone. CAVEAT (08-22): the 5th verdict (`cw-dep-bcgait4-phasedir1`)
  is ENV-CONFOUNDED — it trained on the convention-corrupted sim
  (30660b51) — so it needs a re-run on the repaired sim before any
  phase-RL conclusion; the phase INPUT itself kept zero falls, and
  the operator's staged heading curriculum (fb_20260822T003132:
  forward-only -> small heading set -> full headings -> irregular) is
  UNTRIED.
- BANK BREAK ROOT-CAUSED + REPAIRED (08-22): 30660b51's
  absolute-tibia knee convention + new default stand home leaked into
  the femur-relative sim (full semantics bank 43 FAIL). Fixed at the
  boundary by `linux_control/sim_gait_compat.py` + a
  `_default_plant_deg` guard; hardware untouched. Residue: 7 FAILs
  (rise_valid_plant, rise_rock, trans_drag, getup_honest_ordering,
  recover_floor_rungs, fastprof), all true tibia-150 recalibration.
  3/7 CLOSED same day by re-measuring stale thresholds (no behavior
  change): trans_drag_honest_rise (rise-curl drag 463->656mm,
  `reward.drag_trans_allow_rise_m` 0.55->0.75), rise_rock_feedback
  (leveled peak 4.6-6.7->8.6deg, bound 8->9deg), recover_floor_rungs
  (tangle_70/80 gap margin 2.0->1.5mm, still strictly monotonic).
  ALL 3 REMAINING CLOSED 08-22 with root-cause fixes (not blind
  re-measurement — the operator's own bar for these last items):
  `rise_valid_plant`/`score_replay` shared one cause — PLANT_SPEC's
  height check was comparing the demonstrated (tibia-150-correct)
  rise to a STALE commanded target: `goal.rise_height_mm=[108,114]`
  + `actions.max_height_mm=115` were the PRE-tibia-150 (128 mm
  tibia) belly->plant height gain, never updated when the tibia grew
  ~22 mm. The reference deterministically settles at h_rel=131.94 mm
  (all 3 seeds, open-loop so seed-invariant) — every other PLANT_SPEC
  check (attitude/feet-down/no-flag/support/footprint/current) was
  already clean, only `height_ok` tripped, ~24 mm outside the stale
  target's 15 mm window. Recalibrated the TARGET (not the tolerance)
  to `[128,136]`/`137` — brackets the physically-measured settled
  height (consistent with GETUP's own ~22 mm rigid-FK sag finding at
  this geometry) with margin either side of the 15 mm window. Scoped
  to the two failing banks' own override dicts
  (`RISE_OVERRIDES`/`SCORE_OVERRIDES` in `test_task_semantics.py`);
  `LOWER_OVERRIDES` and the other rise_height_mm call sites
  (`test_bc_anchor.py`, `test_rise_start_bank.py`) were left
  untouched — not broken, no need to touch. `getup_honest_ordering`
  CLOSED separately (root-cause, not re-measurement): a genuine
  partial rise (held mid-ramp crouch, feet loaded) earned LESS than
  freezing at the untouched spawn pose (-12.16 vs -11.26, 3 seeds)
  because the one-shot progress-ratchet credit
  (`reward.getup_k_progress`, was 60) for the honest potential gain
  (~+10) didn't clear the EXTRA gyro/action/current regularizer cost
  of actually executing the rise motion that freezing never pays
  (measured other-than-prog totals -21.6 partial vs -11.7 freeze,
  seed 11) — those regularizers are intentionally untouched (they
  price real physics, not this task's shape). Recalibrated
  `getup_k_progress` 60->200 (partial +10.8 > freeze -11.5, >=20-pt
  margin; swept 60->500, every other GETUP-bank ordering incl.
  flagleg/stilt/thrash preserved). Bank now 152 pass / 1 known-red
  (fastprof, separate fast-gait item, untouched) / 4 skip / 1 xfail —
  the rise-bank residue from the tibia-150 break is fully closed.
  `extract_rise_ref.py --blend-mode ik` (foot-anchored FK/IK blend +
  fresh-seed robustness validation, replacing the raw joint-lerp that
  fell on every seed 0-99) is BUILT+TESTED but did not ship a new
  reference: the best candidate from the pre-tibia-150 stance
  checkpoint (`ppo_goal_cw_stance_dr10`) still nets MORE bank
  failures than the current file because that checkpoint's crouch
  pose is itself asymmetric at the new geometry (blend method can't
  fix a bad source shape) — reverted, not shipped. The circular
  blocker (a tibia-150 stance retrain needs the bank green, but a
  training launch needs the retrain) is now UNBLOCKED on the bank
  side: the bank is green, so a tibia-150 stance retrain arm can be
  spec'd and launched whenever joystick/amp GPU budget allows it.
- MEASURED-PLANT GATE BREAK (08-22): the download hierarchy
  HARD-FAILS the interactive session gate at tibia-150 (sit
  tilt_pitch + reverse tilt_roll falls, fwd yaw -21.8 deg) while the
  matched 128 mm control PASSES — the plant correction alone breaks
  the shipped answer; DOWNLOAD_ANSWER's n=600 numbers are old-plant
  facts. Fix arm `cw-dep-bcgait1-plant150-1` PASSED (core): 0/6 falls
  DR-0+own-DR, gait_valid 6/6, session back-fall gone, fwd yaw
  -21.8->-10.6deg (soft threshold ±10deg, narrow miss); promoted as
  the walk half. `cw-stand-footlow2-plant150-1` (the stance-half fix)
  VERDICTED FAIL 08-22, but half of it is a CONFIG BUG, not a
  behavioral regression: rise/det 1/6, rise/sto 0/6 vs the 128mm
  parent's 5/6 det, 6/6 sto — however this run's launch copy-pasted
  the PRE-tibia-150 `actions.max_height_mm=115`/
  `goal.rise_height_mm=[108,114]` unchanged, never picking up the
  SAME 08-22 recalibration (`[128,136]`/137) that
  `test_task_semantics.py`'s RISE_OVERRIDES/SCORE_OVERRIDES already
  use — CORRECTING THE FIX'S OWN SCOPING NOTE ABOVE: "other
  rise_height_mm call sites... not broken, no need to touch" is WRONG
  for real training/eval launches, which hand-copy the same stale
  pair and are not covered by the test file's override dicts.
  Re-evaluating the SAME frozen checkpoint with the corrected cfg
  (zero retraining) turns 3 of 4 rise start-kinds clean: bridge/
  crouch/flat land within 1.4-3.9mm of the corrected target (det
  1/6->3/6, sto 0/6->4/6) — the policy had been overshooting the
  stale target by +18 to +29mm, almost exactly the tibia-150 height
  delta. Genuine residual defect, unmoved by the cfg fix: RSI-reset
  starts (DeepMimic-style mid-ramp spawn) fail every episode (5/5,
  ~22-29mm undershoot, roll peak up to 9deg vs 0.5-0.8deg elsewhere)
  — RSI's target already tracks the reference file's own (correct)
  height, not the buggy cfg, so the cfg fix can't move it; RSI passed
  fine at the 128mm parent (4/5). hold/lower unaffected (6/6 both
  ways). Follow-up `cw-stand-footlow2-plant150-2c-heightfix` (train-5)
  continues 10M steps from this checkpoint with ONLY the corrected
  cfg. RULE for any future launch of this stance lineage at
  tibia-150: use `actions.max_height_mm=137`
  `goal.rise_height_mm=[128,136]`, never the old 115/[108,114] pair.
  Evidence: `logs/ckpt_eval/plantgate_tibia150_session/`,
  `logs/ckpt_eval/cw_dep_bcgait1_plant150_1_{gate,owncfg,session}/`,
  `logs/ckpt_eval/cw_stand_footlow2_plant150_1_{gate,correctedheight}/`.
- AMP M0 AUDIT + FIRST CODE (08-22): the GPU/Warp trainer
  (`train_ppo_mjx.py`) already had GRU/history/transformer actors and
  most of AMP §6's joystick-command shape (`walk_task.py`'s
  `stress_mix` sampler) before this track's charter was even adopted;
  the one confirmed M0 gap — asymmetric (privileged) critic — is now
  ported (`--asym-critic`, additive-only, verified on-pod: checkpoint
  loads as `AsymActorCriticPolicy`, privileged_idx correctly masks the
  2 measured-velocity obs dims on the actor path). Discriminator,
  motion library, replay buffer, fault injection, and the joystick
  eval suite are confirmed NOT started (zero lines exist).
- PHASE-RL NOISE-BAND LEVER 6 (08-22, `cw-dep-bcgait4-phasedir4-
  entanneal`): FAIL, worse than phasedir3. Annealing PPO's `ent_coef`
  10x (0.000951->0.0001, confirmed monotone in wandb) barely moved a
  WARM-STARTED policy's action std (0.368->0.352 over 2M steps) —
  the entropy bonus is too small a fraction of PPO's total loss to
  drag a loaded log_std down. Clone-relative progress fell to 0.830x
  (cap 0.9x, worse than phasedir3's 0.897x) and slip rose to 1.518x
  (cap 1.15x, worse than phasedir3's 1.41x); zero falls, gait 6/6.
  BUILT: `train_ppo_mjx --warm-log-std-override <logstd>` — a direct,
  guaranteed-to-move lever that forcibly resets a warm-started
  policy's log_std parameter(s) right after `--init-from` loads
  (plain-MLP `log_std` and gru-experts `_log_stds()`), default off/
  no-op, verified on-pod (`-2.0` -> std=0.135 exactly on all 18 dims).
  `cw-dep-bcgait4-phasedir5-stdoverride` (train-0) tests it directly.
- PHASE-RL NOISE-BAND THEORY REFUTED + ROOT-CAUSED (08-22, verdicted
  after dig-in): `cw-dep-bcgait4-phasedir5-stdoverride` FAIL on slip
  only (1.590x clone, cap 1.15x) with std held 0.13 all run; progress
  0.984x clone (first-ever pass) and dir_err 0.794x PASS, zero falls.
  Dig-in found the actual mechanism: SLIP-FINANCED PROGRESS — all
  phasedir arms converge on a lower-cadence/longer-stride/duty-skewed
  gait (swings/leg ~30->~22, stride +25%, leg5 duty 0.56-0.61 vs
  tripod-A 0.42-0.47; swing_s-vs-slip r=0.75, zero cluster overlap
  with the clone) that drags loaded feet to buy progress, because the
  loadslip band (ok=7.0, sized to the std-0.368 clone's noisy sto
  slip 4.8-6.4) never re-tightened after the std override: W&B
  rollout ratio 4.2-4.7, factor ~0.99, excess ~-0.01/step — slip was
  economically FREE while k_walk_course paid the progress. 08-21
  MISALIGNMENT, not exploration noise and not an RL dead end. Cheat
  encoded: `test_phasedir_*` loadslip-band pins in the bank (scripted
  twins can't reproduce the drag regime — IK-clean TripodGait tops
  out at ratio 1.84; the real cheat checkpoint is the bank behavior)
  + matched-env pod pricing A/B `logs/ckpt_eval/pd5_newband_ab_*`.
  Fix arm phasedir6 = band retighten only, A/B-gated.
- DRAG-STANCE CHARGE IS A STEP FUNCTION, NOT A DOSE DIAL (08-22,
  `cw-dep-bcgait4-phasedir7-dragstance` + `-phasedir7b-dragstance-
  halfdose`): after phasedir6 verdicted the loadslip-band VALUE lever
  refuted (FAIL, 1.611x clone slip, ratio never moved), the pricing
  MECHANISM was switched to the already-built `reward.k_drag_stance`
  (per-stance absolute-mm charge, bank-tested 08-11). At k=8000: real
  bite on slip (1.611x->1.323x clone) and stride length recovered to
  the clone's own value, but progress fell 0.961x->0.779x (below the
  0.9x cap) and speed dropped 0.001 m/s under the [0.06,0.096] floor.
  Halving to k=4000 (phasedir7b) to test the dose-response hypothesis
  produced NO change: progress 0.804x, slip 1.346x, speed 0.0585 —
  statistically identical to k=8000 on every gate axis AND on per-leg
  gait metrics (swing count/leg, duty skew, stride length all
  indistinguishable between the two doses), while W&B
  env/reward_drag_stance and ep_rew_mean both scaled down exactly
  proportionally to k (same fraction of stances pay the charge either
  way). Reads as a STEP FUNCTION: any k_drag_stance>0 flips the
  policy into one fixed second local optimum (vs phasedir6's k=0
  optimum), and something in the course/anchor pricing — not the
  drag charge's magnitude — pins that second optimum's speed just
  under the gate floor. Dosing this lever further is refuted;
  DIG-IN flagged on the course/anchor-vs-drag-charge interaction
  before any further reward edit on this lineage.
- INIT-BASIN SELECTION, NOT CHECKPOINT RECENCY (08-22, dig-in
  resolved on `cw-dep-bcgait4-phasedir9b-stdanneal`, FAIL): the
  std-anneal repair applied to two identical stacks diverged purely
  by INIT. From the raw BC clone (`-9`): best arm of the lineage.
  Continuing the cheat-committed pd8 checkpoint (`-9b`): WORSE than
  pd8 on every det gate axis (prog 0.704x vs 0.770x, slip 1.506x vs
  1.254x, speed 0.054) — it cut the drag bill 4x by shrinking
  per-stance travel under the 24mm ABSOLUTE allowance (translating
  less, not slipping less; video: six legs cycling, walking in
  place). Decisive: `-9b`'s final TRAINING reward is equal-or-better
  than `-9`'s (ep_rew -90 vs -138, rew/tick EMA -0.048 vs -0.141,
  same prog income) — the optimization-regime reward surface is
  ~FLAT across the honest and drag basins, so gradient never crosses
  basins at annealed-low std; init decides which basin gets
  polished. Anneal-from-an-earlier-checkpoint is NOT a general
  escape lever. BINDING RULE for the lineage: apply pricing/regime
  repairs by RE-INIT from a pre-cheat checkpoint (teacher/clone),
  never by continuing a cheat-committed one.
- JOYSTICK GATE HARNESS, evaluator half (08-22):
  `rl_move/sim/eval_joystick_gate.py` (+`test_eval_joystick_gate.py`,
  8/8, no sim) — a reusable, versioned wrapper around
  `eval_checkpoint.py` implementing the DONE gate's randomized 60 s
  multi-segment joystick session (stress_mix full family + 4 s+/-
  jitter resampling, held-out seed base 90000), aggregated across
  DR-0 + own-DR x det/sto into one PASS/FAIL. FIRST READING: the
  phase clone (previously verified only on fixed-heading-per-episode
  panels) FAILS this real session hard — zero falls / gait_valid
  24/24, but slip/m med 15.9 (cap 2.9) and dir_err med 65.4deg (allow
  40deg) once the command actually changes mid-episode
  (`logs/ckpt_eval/phase1_clone_joystick_gate_v1/`). "Passes the
  direction-first curriculum" and "joystick controllable" are
  measurably different bars; RL fine-tuning must close this gap, not
  just the shorter clone-relative phasedir rung gate.
- PHASEDIR9-CONT1 CONFIRMS INIT-BASIN FLATNESS FROM THE OTHER
  DIRECTION (08-22): continuing +4M steps from `-9` (the lineage-best
  2M checkpoint, near-pass) regressed hard on every clone-relative
  axis (progress 0.873x->0.66-0.71x, slip 1.08x->1.6-1.7x, speed
  below the 0.06 floor; zero falls/gait-6-6 preserved) despite W&B
  reward partially recovering after a mid-run collapse (27->-670
  ep_rew, then only to 152-213) — a good init drifted to a worse
  basin, mirroring `-9b`'s bad-init-stays-bad finding. VERDICTED
  FAIL as a continuation (not a re-litigation of `-9`'s own reading).
  `phasedir9-seed17` (same recipe, seed 13->17, no reward change)
  tests whether `-9`'s near-pass reproduces before any further
  lineage budget.
- PHASEDIR9-SEED17: PD9'S NEAR-PASS DOES NOT REPRODUCE (08-22,
  VERDICTED FAIL-as-reproduction): seed17 (identical stack, seed
  13->17) landed at/below pd8's own level (progress 0.727x clone vs
  pd9's 0.873x and pd8's 0.766x; slip 1.27x vs pd9's 1.08x and pd8's
  1.254x), matching the pre-registered prediction-if-false almost
  exactly. Zero falls, gait_valid 6/6, clean video both seeds. Reads
  as pd9's near-pass being partly seed luck on top of the log-std-
  anneal fix, not a reliably repeatable recipe.
- BC-ANCHOR/PHASE-LOCK FAMILY BOUNDARY: RESOLVED, NO CADENCE GAP
  (08-22 dig-in on seed17; tool `rl_move/sim/trace_bc_cadence.py`,
  evidence `logs/ckpt_eval/pd9seed17_bc_cadence_trace/trace.json`).
  Per-tick trace of policy vs BC clone vs RAW un-phase-locked
  scripted teacher in the identical env (DR-0 det, pinned 0.08 m/s
  forward): ALL THREE cycle at 0.76 s == the bc_target's own period
  == the 0.75 s phase-obs clock (TripodGait.period=0.75 ==
  1/walk_phase_hz=1.333 by construction — the teacher's cadence is
  speed-INDEPENDENT). The "realized swing_s_mean ~30% slower than
  the teacher" premise was a CONTACT-SEGMENTATION ARTIFACT: the
  clone's 0.25-0.27 s "swing" comes from double lift-offs per cycle
  (24-36 lifts/leg/15 s vs ~19 true cycles; lift-to-lift median
  0.44 s vs 0.76 s), which SPLITS its swings; the RL policy single-
  swings cleanly (19-24 lifts/leg) at 0.372 s vs the raw teacher's
  realized 0.345 s (+8%, minor). Supervision is honest in the eval
  regime: policy-vs-bc_target action MSE 0.00136 (the clone's own is
  4x worse, 0.00549), realized-vs-target xcorr ~1.00 at the same
  ~3-tick (0.12 s) servo lag as the clone. LEVER CLASS CLOSED:
  `train.bc_anchor_coef` dose, `goal.walk_phase_hz`, and phase-lock
  plumbing are all exonerated — do not spend arms there. Failing
  seeds' residual rung deficit is loaded-foot slip during stance at
  MATCHED gait timing/stride/duty (seed17 slip 2.85/m vs clone
  1.89), i.e. the pinned slip-financed-progress family; the
  seed13/seed17 divergence is the init/seed-basin lottery on the
  ~flat reward surface, so the next budget unit is the recipe's
  SEED PASS RATE (longrun29 running, longrun23 queued), then promote
  the best det passer. Also: `env/walk_anchor_frac` is the anchored-
  stance INCOME-GATE fraction (walk_task), not a supervision-
  coverage metric — do not read it as BC-anchor health.
- PHASEDIR9 BUDGET LEVER: SEED-DEPENDENT, NOT CLOSED (08-22,
  `phasedir9-longrun13` FAIL / `-longrun17` CORRECTED TO PASS
  (partial) — see correction note below): 2x the converged-regime
  steps (fresh re-inits from the raw BC clone, same stacks/seeds as
  stdanneal/seed17, `--steps` 2M->4M, `--log-std-anneal-frac`
  0.6->0.3, anneal still ending ~1.2M, ~2.8M steps run converged vs
  ~800k). `longrun13` (seed13, near-passed at 2M) got WORSE (progress
  0.873x->0.792x clone, slip 1.08x->1.286x) — confirmed, unaffected
  by the correction below. `longrun17` (seed17, missed at 2M) was
  FIRST verdicted FAIL (~0.72x/1.32x) by a premature-verdict race —
  that verdict was written at 14:03:08/14:07:02, BEFORE its own eval
  finished syncing (14:03:45/14:04:04) — and did not match the real
  data. CORRECTED (triple-confirmed: on-disk report.json, `ops.sh
  report`, and the run's own W&B `eval/dr0/walk_det/*` summary all
  agree): `longrun17` is a DET-mode PASS of the rung-A clone-relative
  gate — the FIRST in the entire 34+-arm phasedir1-9/longrun lineage
  — at both DR-0 (progress 1.02x clone, slip 0.74x, speed 0.069 m/s)
  and its own DR-0.35 (0.94x/1.01x/0.067, thinner margin). Zero
  falls, gait_valid 6/6, clean roll, no sacrificed legs, clean 6-leg
  video both runs. So the budget lever is NOT uniformly closed —
  it is seed-dependent: identical recipe, opposite basins. DIG-IN
  state: (a) reproduce `longrun17` independently before any
  promotion/champion-append — `longrun29` (seed29) RUNNING,
  `longrun23` (seed23) queued; (b) the family-boundary question is
  RESOLVED (bullet above): supervision exonerated, the divergence is
  the init/seed-basin lottery, so treat this recipe as a measurable
  seed lottery and decide promotion on the n=4 pass rate.
  PROCESS NOTE: a run's ledger verdict must wait for its eval's
  SYNCED marker (per the orchestrator prompt's own instruction) —
  this is the concrete failure mode that instruction exists to
  prevent; a second concurrent-cycle "sync" commit inherited the
  stale numbers without independently re-checking the by-then-
  available real data, so the race survived one extra hop.
- PHASEDIR9 N=4 SEED SAMPLE COMPLETE: 1/4 PASS, "MORE SEEDS" LEVER
  CLOSED (08-22, `longrun23`/`longrun29` both verdicted FAIL):
  seed23 (0.818x progress/1.175x slip clone) and seed29 (0.740x/
  1.296x, worst of the 4) both land in the same 0.72-0.82x/1.15-1.3x
  regression basin as seed13, confirming `longrun17`'s pass was the
  1-in-4 outlier, not the recipe's typical outcome. Zero falls,
  gait_valid 6/6, clean video all 4 seeds. Pass rate too thin to farm
  by seed count alone; the next budget unit is pricing that survives
  the optimization regime, not more seeds.
- AMP M2 STATUE MISALIGNMENT ROOT-CAUSED (08-22 dig-in, both -c1
  arms verdicted MISALIGNED, Wave-1 NO-GO on legacy pricing): the
  frozen half-tripod (triad 0,2,4 planted, 1,3,5 airborne) is the
  legacy walk reward's true optimum for a from-scratch policy —
  statue income ~1.9/tick (rise_finish ~0.85 + posture/height
  kernels ~0.6 + the sigma-0.05 velocity kernel paying ~0.45/tick to
  v=0 across the stress_mix low/stop command fraction) vs realized
  locomotion income ~0.05/tick; 100% of the 38M reward rise was
  statue-polishing (rise_finish 0.09->0.86, walk_speed flat). The
  AMP mechanism was healthy throughout (d_real 0.97 vs d_fake -0.96,
  never saturated) but 0.5 x style_reward 0.06 = ~0.03/tick cannot
  outbid the statue — a discriminator cannot rescue a misaligned
  task reward. Cheat encoded: `test_slipwalk_stork_statue_is_priced_
  out` (stork -238/ep vs gait +558 under the freeprog stack, bank
  6/6 PASS, commit 911fcedf).
- AMP M2 FREEPROG SUICIDE ECONOMICS ROOT-CAUSED (08-22 dig-in, both
  `cw-amp-m2-freeprog-{noamp,style05}` FAIL final): the repriced
  anti-slip stack's third failure mode (rapid topple, neither freeze
  nor walk) is a PRICING DEFECT — per-tick charges ~-1.4 to -3/tick
  with `reward.term_penalty=0` made death FREE; a scripted 1 s topple
  nets +19/ep vs park -243 / stall -143 / skate -1023, i.e. dying was
  the best-paying behavior in the bank short of real walking. Both
  arms LEARNED survival first (ep_len 28->310) then flipped to fast
  death in q4 (tilt terminations 59->132 / 90->241, ep_len falling,
  ep_rew "recovering" purely by episode shortening) at CONSTANT std
  0.367 — refuting log-std anneal AND charge ramp-in as fixes (the
  flip happened at constant std; charges were maxed from step 0 yet
  survival was still learned mid-run). style05's q4 reward recovery
  is confirmed faster death; AMP itself stayed healthy but a style
  channel cannot price termination. Cheat encoded:
  `test_slipwalk_toppling_fast_is_not_an_escape` + `reward.
  term_penalty=400` added to SLIPWALK_OVERRIDES (sized > worst-case
  discounted survival cost ~-295; topple +19 -> -381 < park; bit-
  exact for behaviors that survive to truncation; bank 7/7 PASS,
  commit d9554b04). GENERAL LESSON for every harsh-charge from-
  scratch stack: a reward whose alive per-tick income is net-negative
  for an unskilled policy MUST carry a termination charge exceeding
  the discounted worst-case cost of staying alive, or suicide is the
  optimum. Fix pair LAUNCHED (single change, term_penalty=400; noamp accidentally duplicated as -rr1, a free repro replicate):
  `cw-amp-m2-freeprog-term400-{noamp,style05}` — re-runs the Wave-1
  style-vs-control fork (still UNRESOLVED: the suicide basin
  short-circuited both arms before the style gradient could matter).
  Decisive contrast unchanged: `cw-nobc-slipwalk1-r1` (same pricing
  class, no AMP) froze at 2M — style05 stepping where its twin
  freezes/statues is the first real style-vs-control signal.
- TERM400-NOAMP CONTROL VERDICTED FAIL (08-22, + bit-identical
  `-rr1` repro): the suicide fix WORKED cleanly (0/12 terminations
  DR-0 det+sto, both arms — term_penalty=400 does what it was sized
  to do), but the control still fails the walk gate by the
  predicted-if-false branch: median fwd travel 0.026-0.032 m/15s
  (bar 0.10 m), slip 6.4-22.7/m, gait_valid 3-5/6 det. Six legs DO
  cycle (not the earlier pure statue) but never organize into net
  commanded-direction travel — a shuffle-in-place basin. W&B root
  cause: `env/reward_walk_freeprog_pen` (the freeprog cross-track/
  backward charge, the single largest-magnitude reward line at
  ~-1.4 to -1.8/tick) sits FLAT with zero improvement from the end
  of quarter 1 through quarter 4 at a CONSTANT policy std=0.368
  (`--log-std-init=-1.0` default, never annealed in this arm) —
  reward-per-tick-EMA actually worsens (-1.17->-2.84) then
  flattens, so this is flat-reward-and-flat-metric (genuine FAIL per
  the 08-21 ruling), not reward-still-rising. Matches the joystick
  track's own noise-band fingerprint (constant high std never lets a
  policy commit to organized behavior). Follow-up queued same cycle,
  single lever: `cw-amp-m2-freeprog-term400-stdanneal` (`--log-std-
  final=-2.0 --log-std-anneal-frac=0.5` added, everything else
  identical) — tests whether annealing exploration noise down (the
  fix that unlocked phasedir9 on the joystick track) also unlocks
  organized locomotion here, before any freeprog reward-shape patch.
- TERM400-STDANNEAL VERDICTED FAIL, EXPLORATION-NOISE HYPOTHESIS
  CLEANLY REFUTED (08-22, same cycle): annealing std 0.368->0.135
  (train/std confirmed on the exact schedule) did NOT unlock net
  displacement — it made the SAME stationary basin more regular:
  gait_valid improved to a clean 6/6 det AND 6/6 sto (noamp: 3-5/6)
  with tight low-variance slip (10.3-11.5/m det), zero terminations,
  but median fwd travel FELL to 0.005 m/15s det (noamp: 0.026 m) —
  contact sheet shows the robot in the same spot all 10 mid-episode
  frames. `env/reward_walk_freeprog_pen` plateaus at the same flat
  ~-1.1 to -1.2/tick as noamp regardless of exploration noise, and
  `optimization/reward_per_tick_ema` is WORSE here (-3.29 vs noamp's
  -2.84) — a tighter, less-noisy policy just commits harder to the
  same local optimum. CONCLUSION: this narrows the M2 freeprog
  problem from "noisy exploration never organizes" to a genuine
  REWARD-SHAPE defect — `walk_freeprog_score` (pays along-command
  velocity, charges cross-track/backward, no net-displacement floor)
  has a real local optimum at stable in-place marching. Std/anneal/
  entropy levers are CLOSED for this reward family (mirrors the
  joystick track's own init-basin-flatness finding — a flat reward
  surface doesn't move by de-noising alone). Next candidate fix
  (untested): a walk_freeprog analog of `k_walk_idle_charge` keyed to
  episode-window NET DISPLACEMENT rather than instantaneous speed —
  semantics-bank work before any further train.
- FREEPROG STAGE-CURRICULUM REFUTED (08-22,
  `cw-amp-m2-freeprog-term400-stagecurric` FAIL): ramping
  `goal.walk_cmd_stage` 0->2 via `sched.*` (forward-only first,
  verified ramping in W&B) on the byte-identical noamp stack did NOT
  unlock locomotion — det fwd travel med 0.02m (worse than noamp's
  0.026-0.032m), gait_valid 0/6 det, sacrificed rear legs [3,5],
  video = the same sprawled statue; freeprog_pen pinned -1.5/tick all
  four quarters (reward flat + eval flat = stuck mechanism). Even the
  stage-0 forward-only sub-problem never left the statue basin, so the
  "full-mix-from-step-0" analogy to the joystick steer2 fix does NOT
  transfer to from-scratch training. Staging/exploration lever class
  CLOSED for this family (joins term-penalty and std-anneal). The
  net-displacement-floor patch below is DEPRIORITIZED: the SLIPWALK
  bank audit already shows the ranking aligned (creep +108 >> stall
  -143 >> park -244) and `k_walk_idle_charge=20` already charges the
  statue a smoothed travel floor in full — the defect is a basin
  BARRIER (no accessible income gradient from statue to stepping),
  which more charges cannot fix. Untried gradient source launched
  instead (this cycle): AMP style income with the CLEAN teacher_v2
  lib — `-style05-v2` (lib swap only, 0.5/0.5) and `-stylew2-v2`
  (2.0/1.0 so max style income 2/tick out-earns the statue's
  ~-1.5/tick charges); pre-registered new cheat to watch = in-place
  teacher mimicry at high style weight.
- AMP STYLE-DOSE LEVER CLOSED (08-22, both -v2 arms verdicted FAIL):
  `cw-amp-m2-freeprog-term400-style05-v2` (teacher_v2 lib, 0.5/0.5)
  and `-stylew2-v2` (teacher_v2, 2.0/1.0) land in the SAME in-place
  march/shuffle basin as the plain `-noamp` control — det median fwd
  travel 0.03m/15s both (bar 0.10m), slip 8.09/10.45 per m,
  gait_valid 5/6 det + 6/6 sto both, video visually indistinguishable
  from each other and from noamp. The clean teacher_v2 lib retires
  the "corrupted v1 lib" caveat with zero behavior change. AMP stayed
  mechanically healthy both arms (d_real 0.78-0.79 vs d_fake -0.96,
  unsaturated, no in-place-mimicry degenerate loop — style_reward_mean
  stayed pinned low 0.05-0.07 the whole run, meaning the policy never
  even partially learned to imitate) but 4x the style weight only
  moved realized style income from 0.027 to 0.135/tick
  (`env/reward_amp_style`), an order of magnitude short of
  freeprog_pen's flat -1.4 to -1.5/tick either way. W&B reward FELL
  every quarter in both arms (never rising) — genuine flat/declining
  FAILs per the 08-21 ruling, not continue-longer cases. Per the
  pre-registered decision rule (STATUS.md): with BOTH doses statued
  at 2M, the AMP-style-income route is refuted at this discovery
  scale for the freeprog/term400 pricing family across the whole
  tested range (0.5x-2.0x) — the fix is a genuinely new income
  mechanism, not another blend dose.
- AMP M2 TASK-COMPLEXITY ISOLATION LAUNCHED (08-22,
  `cw-amp-m2-freeprog-term400-fixedcmd`/`-seed11`): every M2
  from-scratch arm to date (noamp/style05/stylew2/stagecurric) threw
  the full randomized `stress_mix` command stream (1.75s resample,
  yaw, 15% stops) at the actor from step 0 or within a ramp window —
  none held the task at its simplest for the FULL budget. This pair
  replaces the goal.* command config with the SLIPWALK bank's own
  literal fixed command (vx=0.05 m/s constant, no heading/resample/
  yaw/stop; reward stack and everything else byte-identical to
  -noamp, no AMP) — the exact setup the bank already proves ranks
  real travel far above stationary behaviors. BOTH SEEDS VERDICTED
  FAIL (08-22, n=2 agree): seed7 det gait_valid 0/6 (legs 2,3,5
  sacrificed every episode, fwd med 0.05m, slip 7.14/m), seed11 det
  gait_valid 0/6 (legs 3,4 sacrificed, fwd med 0.03m, slip 6.43/m) —
  BOTH worse than the harder stress_mix arms (which sacrificed only
  0-1 legs, gait_valid 5/6 det). Same freeprog_pen (~-1.37/tick) and
  walk_gait_min (~0.29) plateau as -noamp in W&B; reward fell every
  quarter both seeds, never rising. TASK-COMPLEXITY HYPOTHESIS
  REFUTED: simplifying the command distribution to the bank's own
  literal simplest validated case did not help — it let MORE legs go
  idle/statue. The from-scratch marching/statue basin for this
  reward family is a genuine PPO exploration/optimization pathology,
  independent of command complexity. RSI (`goal.walk_gait_start_frac`)
  is NOT a fresh lever here — `cw-gait-rsi1` already ran it at
  frac=0.5 on `--impl warp` (08-11, an older pre-freeprog/term400
  from-scratch stack) and was refuted for the identical freeze/statue
  signature; a concurrent cycle read that as closing RSI for this
  family too and bank-tested the actual next mechanism instead —
  `reward.k_walk_swing` (any-direction lift-swing-touchdown bonus) on
  `SLIPWALK_SWING_OVERRIDES` (`test_task_semantics.py`, 11/11 passing,
  commit 1fb65603). **LAUNCHED 08-22**: `cw-amp-m2-freeprog-term400-
  swing-{noamp,style05}`, single-lever `reward.k_walk_swing=1.0` on
  top of the verdicted -noamp/-style05-v2 configs, 2M discovery,
  RUNNING. Bank basis: real gait/creep income up ~11-23%
  (558->622, 103->126) while stall/park/skate AND a new realistic
  farming twin (`shuffle`: genuine six-leg strides reversing
  direction every 0.6s, ~0 net travel) all stay priced below park.
  Pre-registered live-monitoring-only cheat (not bank-cleared, no
  scripted twin could trigger it): single-leg farm (one leg cycling,
  five static, `env/reward_swing`>0 with fwd travel still ~0.02-
  0.03m) — FAIL on sight regardless of return if seen.
- FREEPROG-EMA REUSE TESTED + REFUTED, ZERO GPU SPENT (08-22): the
  obvious cheap fix — feed `reward.walk_kernel_vel_ema`'s already-
  validated stride-averaged velocity into `walk_freeprog_score`
  instead of the instantaneous one (mirrors the phasedir7 kernel fix
  exactly) — was built (default off, bit-exact) and checked on the
  scripted SLIPWALK creep/stall twins BEFORE any training run: it
  NARROWS the creep-vs-stall separation (raw gap 267.7 -> EMA gap
  226.4), the opposite of the prediction. Banked as a measured
  refutation (`test_freeprog_ema_creep_vs_stall_gap_measured`) so no
  future cycle re-derives it; confirms the lead above (needs a
  net-DISPLACEMENT floor, not a smoothed-velocity swap) — velocity-
  domain fixes are the wrong shape for this defect.
- DRAG-STANCE-ALLOWANCE RAMP BUILT + REFUTED AS A GENERAL FIX (08-22,
  3-seed consolidated): `reward.drag_stance_allow_ramp_steps/_mm`
  (new, bank-tested `test_drag_allow_ramp.py` 6/6, mirrors
  `bus.profile_ramp_steps` exactly: cfg-armed/trainer-driven/default
  off/bit-exact) anneals the det-calibrated 24mm drag allowance from
  a loose 48mm (sized to the pd8 dig-in's measured noisy-honest
  stance-travel tail) down to 24mm over the same 1.2M-step window as
  the log-std anneal, so noisy early exploration is never taxed
  harder than the drag cheat itself — a well-motivated regime-gap
  repair. Tested on all 3 of the phasedir9 n=4 sample's FAILing seeds
  (13/23/29): seed13 IMPROVED both axes (0.792x/1.286x->0.831x/
  1.159x clone), but seed23 (0.818x/1.175x->0.792x/1.217x) and
  seed29 (0.740x/1.296x->0.725x/1.466x) both WORSENED on both axes.
  1/3, not a general pricing fix — same seed-dependent-basin-
  selection signature as every other lever in this lineage (the
  ramp changes WHICH basin noisy exploration lands in, it does not
  uniformly reprice toward the honest gait). Do NOT arm as a
  lineage-wide default off one good seed; do not spend further
  budget on this exact ramp schedule without a new idea for why
  seed13 differs. The mechanism itself (the ramp CODE) stays banked
  and available — only its blind reuse across seeds is refuted.
  CLASS CLOSED 08-22 ~17:1x after the 4-arm dose grid
  (`allowramp2slow-seed23/29` = ramp 1.2M->2.4M;
  `allowramp2wide-seed23/29-b` = start 48->64mm): 3 arms worse on
  BOTH clone-relative axes than both their baselines (0.62-0.65x
  prog, 1.65-1.74x slip), 1 nominal improver (slow29 0.78x/1.25x,
  prog delta at the edge of 6-ep noise, slip delta inside noise).
  Across 6 ramp arms x 3 seeds every dose helps exactly one seed —
  basin lottery, no dose-response; best ramp reading anywhere
  (seed13 0.830x) still far from the 0.9x rung. Judgment closure
  (the strict pre-registered 0-for-4 trigger did not literally fire
  because of slow29's noise-edge read — recorded honestly). Next:
  sto gap (stotight grid) + matched-timing stance-slip mechanism.
  Also built: `reward.term_penalty_ramp_steps/_init` (same pattern,
  opposite direction — anneals a termination charge UP from lenient
  to the validated deterrent instead of an allowance down; targets
  the AMP freeprog-stall explore-vs-survive tension, not yet
  deployed pending a sharper diagnosis than "exploration risk" — see
  the freeprog reward-shape finding above).
- direction_err_mean_deg has a ~35 deg tick-level floor from stride
  sway — judge deltas vs a matched clone, not raw values.
- Every pre-08-22 checkpoint (incl. the download hierarchy) trained
  on the old 128 mm plant; cross-plant comparisons need matched
  controls.
- Fallback deployable answer: `footlow2_hard1` stance +
  `bcgait1_hard1` walk + session controller (det 0.967, sto 0.853,
  n=600 — old-plant numbers; see the gate break above;
  `rl_docs/DOWNLOAD_ANSWER.md`).

## Real robot facts

- The robot is off the bench; all physical items are operator-owned.
  No physical motion without an explicit operator ask, ever.
- Scripted tripod gait walks, crabs, and turns on hardware with
  visible loaded-foot slip and roughly half commanded travel.
- Servo/control facts: 25 Hz loop, 1.5 deg/tick stateful slew, loaded
  settle ~250-325 ms, air settle ~9 ms. Working gaits rock 10-20 deg.
- Robot-control/web edits use `linux_control/dev_loop.sh` helpers
  (`make robot-check` / `robot-unit-check` / `robot-status` /
  `robot-deploy`); these never move the robot.

## Policy and eval facts

- Policies output 18 raw joint targets through the SafetyLayer.
- Video/physical behavior outranks reward alone; a reward-passing
  cheat is a metric bug, not a skill.
- Matched-parent controls are mandatory for injected physics/sensor
  axes.
- MJX/Warp GPU stack: 12 single-H200 train pods, ~4096 envs each,
  per-world model DR, canary probes, eval/video logging, desync.
