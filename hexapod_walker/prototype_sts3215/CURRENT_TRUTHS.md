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

## Current top rulings (operator, 08-21)

- Bad evals/canaries with training reward still rising is NOT a fail:
  continue the run longer and/or align the reward with the evals
  (`RUN_INTERPRETATION_RULES.md`). A known exploit on video means
  misalignment to repair, not a lineage kill.
- No operator pauses: assume-and-go with recorded assumptions. Only
  physical-robot access and spend approvals wait.
- While either gate is unmet, an idle fleet next to an empty backlog
  is the failure state; build tools, fund continuations, queue the
  next milestone arm.
- SIM SPRINT and the seven-track structure are superseded.

## Facts that feed the two tracks

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
  STILL RED: rise_valid_plant/score_replay (PLANT_SPEC height-window
  miss on an otherwise-clean final pose), getup_honest_ordering
  (partial-crouch now pays less than freezing — a real pricing
  defect), fastprof (separate fast-gait item). `extract_rise_ref.py
  --blend-mode ik` (foot-anchored FK/IK blend + fresh-seed robustness
  validation, replacing the raw joint-lerp that fell on every seed
  0-99) is BUILT+TESTED but did not ship a new reference: the best
  candidate from the pre-tibia-150 stance checkpoint
  (`ppo_goal_cw_stance_dr10`) still nets MORE bank failures than the
  current file because that checkpoint's crouch pose is itself
  asymmetric at the new geometry (blend method can't fix a bad source
  shape) — reverted, not shipped. CIRCULAR blocker: the real fix is a
  tibia-150 stance retrain (mirroring the walk fix arm), but that is
  itself a reward-mechanism launch gated on the same 2 still-red rise
  tests, which don't need a training run to fix. Next: root-cause the
  PLANT_SPEC height-window and getup partial-crouch pricing directly.
- MEASURED-PLANT GATE BREAK (08-22): the download hierarchy
  HARD-FAILS the interactive session gate at tibia-150 (sit
  tilt_pitch + reverse tilt_roll falls, fwd yaw -21.8 deg) while the
  matched 128 mm control PASSES — the plant correction alone breaks
  the shipped answer; DOWNLOAD_ANSWER's n=600 numbers are old-plant
  facts. Fix arm `cw-dep-bcgait1-plant150-1` PASSED (core): 0/6 falls
  DR-0+own-DR, gait_valid 6/6, session back-fall gone, fwd yaw
  -21.8->-10.6deg (soft threshold ±10deg, narrow miss); promoted as
  the walk half. `cw-stand-footlow2-plant150-1` stays blocked on the
  rise-family residue above. Evidence:
  `logs/ckpt_eval/plantgate_tibia150_session/`,
  `logs/ckpt_eval/cw_dep_bcgait1_plant150_1_{gate,owncfg,session}/`.
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
