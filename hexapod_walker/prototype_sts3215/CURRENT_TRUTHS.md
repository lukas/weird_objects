# CURRENT TRUTHS - accepted facts and rulings

Last compacted: 2026-08-23 (third-track CPG update; folded the 08-22
plant-gate/bank findings). Accepted current facts, not narrative. If
history or old prose disagrees with this file, this file wins.

## Mission

Four goals until all gates are green (operator 08-23 updates): (1)
`joystick` — RL from the scripted programmatic gait to
joystick control, gated on 60 s of command-following in MuJoCo with
zero falls and teacher-band slip; (2) `amp` — the from-scratch AMP
program in `rl_docs/AMP_LOCOMOTION.md` (no Isaac Lab; MJX stack;
build all tools; done at M5 MuJoCo transfer); (3) `cpg` — a
Berkeley-style low-dimensional gait-search path that directly
optimizes parameterized SE2/CPG walking against the behavioral gate,
with adoption into joystick/AMP only through measured A/B forks;
(4) `walkcurr` — prior-free walking curriculum (Kawawa-2022 lineage,
registered by operator order 20260823T154657Z): from-scratch PPO,
walk-only diet, fixed-forward-first rung ladder, bank-proven reward
ranking before every mechanism launch.
Operator-launched out-of-scope runs get honest triage but no agent
follow-ups.

## Current top rulings (operator, 08-21/08-22/08-23)

- WALKCURR TRACK REGISTRATION (operator 08-23, focus note
  20260823T154657Z — binding): the fourth first-class track. Rules:
  (a) every rung trains WALK-ONLY (`goal.walk_pure=1`) — the
  cw-kawawa2022-pf-flat1 failure (hold/raise/track/unload carrying
  aggregate reward while walk died) must be impossible by
  construction; (b) fixed forward 0.05-0.06 m/s first, headings/
  direction-changes/DR only as later rungs after certified passes;
  (c) reward-mechanism launches require the WALKCURR_PF bank green
  (clean commanded walking > park/stall > sideways/reverse/wrong-way
  > high-slip/skate/fall under the run's exact cfg); (d) BINDING
  TRIAGE RULE: reward rising while walk eval flat/down or walk
  terminating = misaligned -> stop same-recipe seeds/continuations,
  audit reward/eval/simulator. FACT (bank measurement 08-23): the raw
  kawawa launch stack was misaligned for the walk goal ALONE — park
  +387 out-earned clean walking +325; the v2e re-pricing (freeprog
  3.0 EMA, step_event 1.0, park_duty 4.0, idle 2.0, heading 0.5,
  loadslip_excess 4.5, term_penalty 1200) measures the required
  ranking exactly (gait +346 > stall -31 > park -352 > sideways -609
  > reverse -741 > skate -1058 > topple -1164). FACT: desktop temp
  commit b126ceb3 (RecurrentPPO/LSTM trainer support) is LOST; the
  canonical recurrent path for later rungs is `--gru`; rung 1 is
  memoryless MLP 128/64/32 + ELU (`--activation-fn`, landed 08-23).

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
- While any track gate is unmet, an idle fleet next to an empty backlog
  is the failure state; build tools, fund continuations, queue the
  next milestone arm.
- SIM SPRINT and the seven-track structure are superseded.
- BERKELEY/LEVINE MICROROBOT PAPER ADAPTATION (08-22): the relevant
  lesson is NOT "try another PPO seed." Yang et al. optimize a small
  CPG parameter vector with BO/contextual BO and judge the trial on the
  same final displacement/drift/energy objective used for selection.
  Local equivalent is `rl_move.sim.paper_cpg_search`: search
  `SE2FootGait` timing/clearance/workspace parameters in MuJoCo, using
  `sim_gait_compat.SE2FootGait` so knees cross the sim-relative
  boundary correctly, and score the run directly on progress,
  cross-track, loaded-foot slip, falls, tilt/height, and effort. Run
  straight 50 x 20s first, then contextual headings/turns 250 x 20s
  only if straight improves under the same metric.
  **VERIFIED + EXTENDED (08-22 ~21:0x kick cycle):** (1) the straight-50
  winner (trial 43: tetrapod period=2.0, swing_frac=0.18494,
  lift_m=0.035, cmd_tau=0.65225, workspace_margin=0.92865) reproduces
  BIT-EXACT on the controller (prog_frac 0.9028352617863246, slip/m
  0.5368772921546681, 0 falls) — the rollout is DETERMINISTIC (no reset
  randomization at mu=0), so seed replays are vacuous; robustness =
  off-axis generalization, which PASSES: ±35°/±70° headings prog
  0.78–0.87, slip 0.58–0.95, cross ≤0.05, 0 falls. The straight result
  is REAL. (2) The operator's run used `--slip-weight 0.9` (score
  0.36455 reproduces exactly at 0.9; default is 0.7). (3) TWO SCORER
  DEFECTS found+fixed before the contextual launch (tag
  `exp/c0822-paper-cpg-yawwrap-slipnorm-replay`): yaw_delta wrapped the
  endpoint difference, so a 20 s turn at 0.2 rad/s (target 4 rad > pi)
  that actually turned +3.9 rad scored as −0.6 of target (sim yaw
  convention verified consistent with the gait by per-leg probe — the
  turns were near-perfect: +0.96/+0.99 of target, yaw_err 0.16/0.03
  rad); and slip_per_m divided by translation progress, saturating the
  slip penalty on pure turns — now normalized by progress +
  0.17·|yaw_delta| (foot-arc). Straight metrics shift <2%. (4)
  Contextual 250 x 20s (5 headings + 2 turns, wz 0.2, slip-weight 0.9,
  GP warm-started with trial 43 via new `--warm-json`) COMPLETED
  250/250 (~21:14, read 21:34):
  `logs/paper_cpg_search/paper-cpg-contextual250-20260822T21Z.json`.
  **Contextual winner (iter 76): tetrapod period=2.0,
  swing_frac=0.2961, lift_m=0.0299, cmd_tau=0.1,
  workspace_margin=0.7759 — score 0.166 vs trial-43's 0.033 under the
  same fixed scorer (5x).** Per-context: headings prog_frac 0.84–0.89
  / cross <=0.043 / slip 0.56–0.63; turns yaw_along_frac 0.993/1.012;
  zero falls/terminations across all 7 contexts. Note the winner
  moved AWAY from the straight-50 champion on 3 of 5 knobs (shorter
  swing hold, faster cmd_tau 0.65->0.1, tighter workspace 0.93->0.78)
  — the contextual objective genuinely prefers a different gait
  parameterization than pure straight-line. score_min -0.205 (worst
  single context still negative under slip-weight 0.9 pricing —
  headroom exists but no context misbehaves behaviorally). The paper
  adaptation question (does BO-on-CPG-params beat the hand-scripted
  teacher on the paper's own objective, straight AND contextual) is
  ANSWERED YES; any use of these params (regenerating the AMP motion
  library / teacher-band refresh) is a separate pre-registered fork,
  not an automatic swap — logged in OPERATOR_QUESTIONS.md
  (q_20260822T2140Z). Operator update 08-23: this is now the third
  first-class track `cpg`; it owns CPG gate hardening, controller
  artifact export, and teacher-v2 A/B adoption experiments.
- **CPG ROBUST GATE FULL PASS (08-23)**: `eval_cpg_gate.py`'s
  held-out 60s robustness panel (DR-0 + a second held-out script +
  friction 1.2x/0.8x + loaded-bench servo fit) is FULLY GREEN for the
  contextual-250-derived winner once a closed-loop yaw trim is added.
  A 120-iteration GP robustness refinement search (warm-started from
  the contextual-250 winner, `--mu-list 0,1.2,0.8` panel scoring)
  could NOT find any open-loop (period, swing_frac, lift, cmd_tau,
  workspace_margin) point that fixes the mu0.8 turn overshoot
  (1.33/1.35 vs the [0.70,1.30] band, identical to the un-refined
  winner) — root cause confirmed as a friction-dependent GAIN error,
  structurally unreachable by more open-loop parameter search.
  `rl_move/sim/yaw_trim.py` (`update_trim`, 8/8 unit tests, pure
  numeric) implements a proportional MULTIPLICATIVE trim on commanded
  omega from measured-vs-commanded yaw rate (1.0s windows, turn
  segments only), wired into `eval_cpg_gate.py --yaw-trim` (default
  off, bit-exact when off). Re-run with the SAME params + `--robust
  --yaw-trim`: all 5 panels PASS (mu0.8 turn ratio 1.33/1.35 ->
  1.07/1.07; dr0/dr0_script2/mu1.2/loaded also tightened slightly;
  zero falls/sacrificed legs everywhere; slip/m 0.69-1.36, inside the
  1.4-2.9 teacher band). Video-reviewed (5 contact sheets): clean
  six-leg cycling, upright, no pathologies. Exported first CPG
  artifact: `rl_move/sim/policies/cpg_controller_robust120_yawtrim.json`.
  This is the track's first full behavioral-gate PASS, but NOT a
  track-closure declaration: the gate text also names a web-UI/
  teacher-library-generator loader for the artifact schema, which does
  not exist yet (grep-confirmed, zero references anywhere in the repo
  besides `eval_cpg_gate.py` itself), and the teacher-v2 A/B adoption
  fork (Next item 3) is unstarted — champions/gates are append-only,
  adoption needs its own measured fork, same discipline as the
  joystick track's promotion-pending-integration precedent. Evidence:
  `logs/cpg_gate/robust120-winner{,-yawtrim}/`,
  `logs/paper_cpg_search/paper-cpg-robust120-20260823.{json,log}`.
  **UPDATE 08-23 (later same day)**: the web-UI loader
  (`linux_control/cpg_controller_loader.py`, `DriveController`
  gait-6/`CPGLIST`/`CPGLOAD`, webui picker, 10/10 tests, zero hardware
  contact) is now built, and the teacher-adoption A/B has a matched
  reading (`cw-cpg-teacherfork-ab-cpgv1-acq1b` vs
  `cw-cpg-teacherfork-ab-style05-budget2`, both +6M from a shared 2M
  base): the CPG-built motion library is CO-EQUAL to `teacher_v2` as
  an AMP style source at matched budget (det progress_ratio 1.35/0.77m
  vs 1.21/0.71m, teacher control ~flat vs its own 2M read — real
  gap-closing by the CPG side, not shared budget lift; sto slip still
  favors teacher). Both named gaps in the track's own DONE-gate text
  are now closed: **track goal MET, work is maintenance-only**
  (hardware drive of the exported controller stays [operator]).
- **AMP M5 GATE GREEN (08-23 ~22:0x — 4/4 seed replication; track
  DONE at M5 per `rl_docs/AMP_LOCOMOTION.md`, M6 hardware is
  [operator])**: the 1.1 Hz cadence recipe
  (`goal.walk_phase_hz=1.1` on the phasehz05/pushcal518 lineage)
  passes the full own-cfg `eval_amp_m5` suite (walk judged at
  wpm24, n_translating=28) on seeds 7/17/23/29 — every section, zero
  falls/terms in every read, strips watched. Bars 0.75 prog / 3.5
  slip / 0.20 tips; reads: seed7 0.893/3.131/0.145, s17
  0.983/2.661/0.114-0.166, s23 0.959/3.294/0.138-0.140, s29
  **1.058/2.883/0.111-0.142** (best). **M5 champion:
  `ppo_goal_cw_amp_m4_turnfault_seq1_pushcont1_tipfrac05_pushcal518_phasehz11_s29.zip`.**
  Cadence dose curve is NON-monotone (0.7-0.9 Hz walk trough); never
  retune below 1.1 Hz without new evidence. Evidence:
  `logs/ckpt_eval/..._phasehz11{,_s17,_s23,_s29}_m5/`.
- **JOYSTICK DONE-GATE DECLARED MET (08-23, assume-and-go, cycle
  c0823-seed37-triage — answering `OPERATOR_QUESTIONS.md`
  q_20260822T1730Z, unresolved for a full day across many cycles that
  kept re-verifying instead of deciding)**: the pre-registered gate
  (60 s randomized joystick command script, MuJoCo, zero falls,
  directions followed, slip/m <= teacher band ~2.9, n>=12 det+sto,
  DR-0 and own-DR) is met, not just approached. Evidence already on
  record before this declaration: `stotight45` recipe (fresh re-init
  of `phasedir9-longrun17`, single lever `--log-std-final -4.5`)
  passes `eval_joystick_gate.py`'s full 60 s held-out `stress_mix`
  session (random_hold/flip_180=reverse/sweep_circle,square=turns/
  stop_go/jitter) on **4 independent training seeds** (17/23/13/29,
  n=48 episodes each: zero falls 48/48, gait_valid 48/48, no
  sacrificed legs, slip/m 2.41-2.78 vs cap 2.9, dir_err 36.4-39.4deg
  vs allow 40deg/teacher floor ~35deg) **and on 2 independent
  held-out command-seed bases** (90000 and 314159) on the seed17
  checkpoint alone. Champion (widest margins): `stotight45-seed13`
  (`ppo_goal_cw_dep_bcgait4_phasedir9_stotight45_seed13.zip`, slip/m
  2.407, dir_err 36.4deg). Honest residual caveats (do not block the
  declaration, the gate's literal text does not name them): own-DR
  sto margins are thin on some seeds (seed29 slip 2.736/2.9); det mode
  is slightly softer than the pre-noise-floor `longrun17` checkpoint
  (progress ~0.85x clone vs 1.02x); the lineage trains with the legacy
  `bc_anchor_knee_abs=1.0` dialect and a +2-dim `walk_phase_obs`
  contract (board runner must implement the same phase clock before
  hardware use); training command was fixed-forward 0.08 m/s only —
  the reverses/turns/stops the gate scores are emergent generalization,
  not directly trained. A follow-up on-distribution command-training
  lever (`cmdmix`) was tried and CLOSED 0/3 PASS (on-distribution
  command training regresses margins on a BC-anchored recipe — see
  `rl_docs/tracks/joystick/STATUS.md`), so no untried margin lever
  remains; further hardening is optional polish, not gate-blocking.
  The operator retains override authority (`OPERATOR_QUESTIONS.md`
  q_20260822T1730Z left open for a reply); absent one, the fleet
  treats this track as **DONE** and concentrates registered-goal
  effort on `amp`/`cpg`.

## Facts that feed the tracks

- **AMP M4/M5: FIRST-EVER FULL `eval_amp_m5` PASS (08-23 ~06:5x)** —
  `cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05` (turn+push+fault
  composition, single lever `goal.walk_turn_in_place_frac=0.5` —
  whole-episode dedicated turn PRACTICE, not reward pricing — vs the
  pushcont1-ypfix1 recipe) passes all four `eval_amp_m5` sections:
  walk (0 terms, gait_valid 12/12), yaw (tip-left/right err
  0.162/0.184, under the suite's own <=0.20 bar and better than the
  fault-only solo parent's 0.18/0.17), push and fault (2/6 det + 0/6
  sto terms, gait_valid 11/12, one legit carried-fault-leg). Clean
  monotonic 3-arm dose grid (frac 0.2/0.3/0.5 -> tip err 0.207/0.234,
  0.201/0.214, 0.162/0.184) confirms a real mechanism, not a lucky
  seed. Root cause: a perfect heading-hold segment earns ~1.7x a
  perfect tip-turn segment's return purely from lower actuation cost
  (not mispricing) while turn-in-place was only ~7.5% of independently
  -sampled training exposure; this exact curriculum lever was tried
  and refuted once BEFORE the BC-turn-clone motor-pattern fix and
  never retried after RL actually had a turning motor pattern to
  practice. CAVEATS (not yet a track-DONE declaration): single seed
  (seed=7) — repro arms in flight (seed13/s2/s3, an acquisition-budget
  continuation); push/fault sections are jointly- not independently-
  tested (both hazards permanently baked into this checkpoint's own
  training cfg); M6 hardware is [operator]-owned regardless. Evidence:
  `logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_tipfrac05_{gate,
  m5}/`, `logs/probe_walk_income/hold_forward_income_ypfix1.json`.
- **AMP M4/M5 CORRECTION (08-23 ~09:5x, n=7 seeds closed)**:
  `eval_checkpoint.py`'s `gait_valid` field is `not sacrificed_legs`
  ONLY — it is NEVER cleared by a TERM, so "own-cfg DR-0 gait_valid
  12/12" does NOT mean zero falls; always check the raw per-episode
  `terminated`/`term_reason` (`ops.sh report` prints `TERM <reason>`
  per line and a `terms N` aggregate — read that, not `gait_valid`
  alone). Re-auditing all 7 completed `tipfrac05` (turn+push+fault,
  `goal.walk_turn_in_place_frac=0.5`) seeds this way: the CHAMPION
  seed7 itself has 2 real falls (walk/det/3 + det/5, both tilt_roll,
  roll_peak 39-41deg) and seed23(s2) has 2 (det/3 + sto/5) — BOTH were
  previously logged as "clean 12/12/zero-falls," which was wrong.
  Corrected tally: only seed43 is genuinely fall-free (6/7 seeds have
  >=1 real fall on the hazard-baked own-cfg DR-0 gate). Sharper
  finding: 5 of 6 falling seeds (7,23,13,31,37) fall at the SAME
  deterministic held-out episode index (`walk/det/3`, `--seed 0`
  fixed so it's the identical command+fault draw for every
  checkpoint), all `tilt_roll`; video shows a stiffly-held/dragging
  leg (the permanently-baked per-episode weak/frozen-joint fault)
  during what reads as a turn-in-place segment, then a clean topple.
  This is a near-universal hard-maneuver failure the recipe sits at
  its margin against, not primarily a training-seed lottery — do not
  promote `tipfrac05` (or fund further acq1/kernel-EMA budget/pricing
  arms on it) as an M5 candidate off any `gait_valid`-only safety
  claim. DIG-IN flagged to replay `walk_det_3.mp4` across the family,
  pin the exact fault+command combo, and check whether it's
  under-sampled in `stress_mix` training (a buildable exposure fix,
  not an n=12-seeds problem). Full detail + evidence paths:
  `rl_docs/tracks/amp/STATUS.md` top banner.
  **FURTHER TRACED (same audit, this cycle): the fall risk PRE-DATES
  turn-in-place AND pre-dates fault, and traces to PUSH.** Raw-
  `terminated` audit of the lineage's own ancestors:
  `cw-amp-m4-turnfault-seq1` (fault only, NO push) — own-cfg DR-0
  gate is genuinely 0/12 falls, clean. The moment push composes in,
  `cw-amp-m4-turnfault-seq1-pushcont1` (fault+push) shows 4/12 REAL
  falls that its own PASS-partial verdict never counted (it read
  "gait_valid 10/12 >= 9/12" as the floor, same metric bug). Its
  `-ypfix1` yaw-pricing respec (unrelated lever, same fault+push)
  roughly halves it to 2/12 falls — also uncounted at the time. An
  even earlier, fault-FREE, push-only M3 checkpoint
  (`cw-amp-m3-pushcur1-noamp-b1530`) already falls 2/12 (det/3
  tilt_roll 40.9deg — the SAME episode index again — + sto/0
  tilt_pitch). **CONCLUSION: push-disturbance recovery itself has an
  uncounted ~15-30% real fall rate across the ENTIRE M3/M4
  push-composition lineage, present before fault injection and before
  any turn-in-place lever — turn-in-place/kernel-EMA work has been
  layered on top of, and is not the cause of, an already-shaky
  push-recovery foundation.** `dr.ext_push_n=(10,25)` N over
  0.15-0.4s, random direction/timing (`domain_rand.py` `RandRanges`)
  — untested whether push magnitude, timing-vs-gait-phase, or
  undertrained recovery is the actual mechanism; that trace is the
  next dig-in step, ahead of any further turn-in-place or kernel-EMA
  arm. Every push-composition verdict logged before this correction
  (`pushcont1`, `ypfix1`, every `pushcur*`/`pushhard*` M3 arm, all
  tipfrac0x/seed arms) should be re-read for raw fall count, not
  `gait_valid`, before being cited as a safety baseline again.
  **CONFIRMED BY DIRECT ISOLATION (same cycle, 2 extra eval-only reads
  on the controller CPU, zero GPU/training spent, checkpoint
  unchanged):** re-ran `pushcont1`'s own 12-episode own-cfg panel
  with ONLY `dr.ext_push_prob` flipped to 0 (fault stays on) — result:
  **0/12 real falls**, every roll_peak <=18.5deg, `walk/det/3`
  specifically drops from a TERM to roll_peak 4.0deg. Re-ran the
  complementary swap (`dr.fault_prob=0`, push stays on) — still falls
  (`walk/sto/0` tilt_pitch) though not at `det/3` this time. **Push is
  the necessary driver of real falls in this checkpoint; fault only
  shifts WHICH episode is vulnerable.** **ANSWERED, same cycle, one
  more eval-only read: it IS a magnitude threshold, not a blanket
  recovery failure.** Halving the push-force range (`dr.ext_push_n`
  10-25N -> 5-12N, fault still on) on the SAME checkpoint: **0/12
  falls, every roll_peak <=8.0deg** (vs up to 41.3deg at full force)
  — clean recovery every time at half force. The push-recovery
  behavior is real and works up to roughly half the trained range; it
  specifically fails somewhere in the 12-25N band. Next dig-in step
  is now precisely bounded: bisect the actual failure threshold
  within 12-25N (a few more eval-only reads, no training needed yet)
  and decide recalibrate-the-range vs. add-recovery-training-exposure
  /pricing at the top of the current range; also worth checking push
  timing vs gait phase once the threshold is pinned. **BISECTED ONE
  MORE STEP (3rd eval-only read): `dr.ext_push_n=12-18N` (upper-mid of
  the original 10-25N range) also lands 0/12 real falls, but roll_peak
  rises to 20.7deg (det/5, close to the 25deg term threshold) and two
  sto episodes start SACRIFICING a leg instead of falling (`sac=[2,4]`,
  `sac=[0]`) — a visible transition zone. Threshold is bracketed:
  clean through ~18N, real falls appear by 25N (the original range's
  own top end) — narrow enough to recalibrate `dr.ext_push_n` toward
  ~(5,18) as an immediate, low-risk mitigation while the
  recovery-training/pricing question is dug into separately.**
  Artifacts:
  `logs/ckpt_eval/diag_pushcont1_{nopush,nofault,halfpush,midpush}/report.json`
  (+ contact sheets/videos, controller-local, gitignored — not a
  ledger-tracked eval, just a diagnostic). **INDEPENDENT REPLICATION on the fully-composed
  CHAMPION checkpoint itself (not just the pre-turn-in-place
  `pushcont1` ancestor), same cycle, 3 more eval-only reads on
  `tipfrac05` (seed7, turn-in-place+push+fault all baked in):
  `dr.ext_push_prob=0` -> 0/12 real falls (max roll 19.3deg, still
  some leaning); `dr.ext_push_n=3-6N` -> 0/12, tightest margins of any
  arm (max roll 14.5deg); `dr.ext_push_n=25-40N` (above the trained
  10-25N ceiling) -> **8/12 real falls** (4/6 det incl. the recurring
  `walk/det/3`, 4/6 sto), roll peaks 22-41deg. Same monotonic
  magnitude dose-response as the `pushcont1` bisection, confirming
  turn-in-place training did not change the underlying push-
  vulnerability profile — it is a property of the push-recovery
  mechanism itself, present unchanged through the whole composition
  chain. Artifacts: `logs/ckpt_eval/pushdiag_tipfrac05_{off,low,high}/
  report.json` (+ contact sheets/videos, controller-local,
  gitignored).
- **PUSH-FORCE RECALIBRATION FIX CONFIRMED ON FRESH RETRAINS, AT BOTH
  COMPOSITION TIERS (08-23 ~10:3x-10:4x)**: recalibrating
  `dr.ext_push_n` from the trained 10-25N to 5-18N (single lever,
  everything else byte-identical, fresh init from `turnfault_seq1`,
  seed=7, 2M) eliminates the real falls found above — not just at
  eval-time on an already-trained checkpoint, but when the model is
  actually TRAINED at the new range. Fault+push tier
  (`pushcont1-pushcal518`): 0/12 real falls vs `pushcont1`'s own
  4/12. FULL turn-in-place+fault+push composition
  (`tipfrac05-pushcal518`): 0/12 real falls vs `tipfrac05`'s own
  2/12 — including a clean pass on `walk/det/3`, the exact episode
  index where 5/6-6/7 of the seed-safety batch toppled. Video-
  confirmed both times: clean upright six-leg cycling, no topple
  frame anywhere; direction_err/slip_per_m stay in the parent's own
  range, no new regression traded for the fix. **This closes the
  root-cause chain opened above: push magnitude (not turn-in-place,
  not fault, not a training-seed lottery) was the actual mechanism,
  and recalibrating the trained range — not more seeds, not more
  pricing/curriculum levers — is the fix.** Still open before any
  M5-candidate/champion promotion: seed-robustness of
  `tipfrac05-pushcal518` (`-seed23`/`-seed13` twins launched
  alongside, unverdicted) and a fresh `eval_amp_m5` cross-engine read
  on the recalibrated checkpoint (the M5 PASS on record is for the
  un-recalibrated `tipfrac05`). Evidence:
  `logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_{pushcal518,tipfrac05_pushcal518}_gate/report.json`.
  **UPDATE (08-23 ~11:0x): that fresh `eval_amp_m5` read is now in —
  `m5_pass=false`, a GENUINE trade, not a clean promotion.** Push
  section improves (0 det terms, was 1) and fault section clears its
  own bar for the FIRST time on this lineage (gait_valid 12/12, was
  9/12 with 2 sacrificed legs — the named blocker on the original M5
  attempt). But walk (det_slip_med 3.67 vs bar 3.5, parent 3.36) and
  yaw (tip_left/right_err 0.2157/0.2351 vs bar 0.20, parent
  0.162/0.184) both slip just past their own strict bars — zero
  falls/terminations in either section, video-clean six-leg cycling,
  a tracking-quality miss against a strict threshold, not a stability
  regression. Also: `-seed23` twin verdicted PASS (0/12 falls,
  matches seed7); `-seed13` still pending (another cycle's pod).
  **Net position: push-force recalibration fixes the two things that
  actually endanger the robot (real falls, fault-carry gait validity)
  at a small cost to walk-slip/yaw-tip margin — do not call this
  checkpoint an M5-candidate base yet.** Evidence:
  `logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_tipfrac05_pushcal518_m5/m5_verdict.json`.
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
  **REPRODUCED 4/4 (08-22 ~18:1x): the recipe passes the DONE gate
  on EVERY tested seed** — seed23 (slip 2.78 / dir 39.4), seed13
  (2.407 / 36.4, widest margins) and seed29 (2.704 / 39.05) all
  pass=true, 0 falls/48, gait_valid 48/48, no sacrificed legs,
  videos watched det+sto both DR. Seeds 13/29 were the lineage's two
  worst basins at the -3.2 dose (1/4 pass rate there): the -4.5
  noise floor converted the seed lottery into a reproducible recipe.
  Seed-robustness is CLOSED; the only honest hardening gaps left are
  own-DR sto margins (thinnest: seed29 slip 2.736/2.9, dir 39.4/40)
  and the shared 15s-rung det progress trade (~0.85x clone).

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
  — RSI passed fine at the 128mm parent (4/5). hold/lower unaffected
  (6/6 both ways). Follow-up `cw-stand-footlow2-plant150-2c-heightfix`
  (train-5) continued 10M steps from this checkpoint with ONLY the
  corrected cfg. RULE for any future launch of this stance lineage at
  tibia-150: use `actions.max_height_mm=137`
  `goal.rise_height_mm=[128,136]`, never the old 115/[108,114] pair.
  Evidence: `logs/ckpt_eval/plantgate_tibia150_session/`,
  `logs/ckpt_eval/cw_dep_bcgait1_plant150_1_{gate,owncfg,session}/`,
  `logs/ckpt_eval/cw_stand_footlow2_plant150_1_{gate,correctedheight}/`.
- RSI-START RSI-TARGET STALENESS ROOT-CAUSED + FIXED (08-22,
  `cw-stand-footlow2-plant150-2c-heightfix` VERDICTED FAIL, exactly
  the gate's own pre-registered branch): the RSI-reset defect above
  was WRONGLY attributed to "RSI's target already tracks the
  reference file's own (correct) height" — it does NOT. 10M more
  training steps left RSI-start rise pinned at 22-29mm height error
  (0/5 combined, zero movement) while bridge/crouch/flat held steady
  (det 3/6, sto 4/6) and training reward flattened after Q1
  (120.6/201.1/198.1/200.6) — reward-flat + eval-flat, a genuine
  stuck mechanism. Root cause: `sim_env.py`'s RSI height-schedule
  rewrite anchors the episode's ABSOLUTE height target to
  `rise_ref_belly2plant.npz`'s OWN recorded final height
  (`ref["h"][-1]` = 110.96mm) — that npz predates the tibia-150
  change; replaying the SAME `q_rad` trajectory on the current sim
  settles at 131.94mm (the exact number `rise_valid_plant` already
  measured for a different bug, above). The ~21mm gap matches the
  observed RSI error almost exactly: every RSI episode was being
  TRAINED toward a target ~21mm below the real
  `goal.rise_height_mm=[128,136]` window the eval grades against —
  a genuine reward<->eval misalignment, not an unlearnable motion.
  FIXED: the RSI schedule now anchors the absolute target to the
  episode's own already-sampled, live-cfg height (correct
  post-tibia-150) and uses the reference array only for FRACTIONAL
  progress at the spawn point (robust to the stale absolute scale).
  2 new regression tests (`rl_move/tests/test_rise_rsi_height_
  target.py`, both pass) lock the fix against recurrence; full
  `test_task_semantics.py` bank re-run clean (159 pass, only the
  pre-existing known-red `fastprof` fails, unrelated). LAUNCHED:
  `cw-stand-footlow2-plant150-3-rsifix` (same checkpoint, ONLY the
  mechanism fix, 10M steps) — tests whether RSI now actually improves
  with an aligned reward target. **VERDICTED PASS 08-22 (~17:5x)**:
  RSI-start rise went 0/5 -> 5/5 (det rsi height err 0.8/10.7/
  11.7 mm, sto 3.2/1.2 mm, all <=15 mm), bridge/crouch/flat 7/7, 12/12
  rise successes overall, zero terminations; training reward rose then
  held — reward and eval now agree, confirming the misalignment
  diagnosis end-to-end. Videos: mid-ramp sprawl -> symmetric rise ->
  level planted stance. Caveats: 2/3 det RSI episodes end "leaning"
  (roll 8.9-9.0 deg, height err at the window edge); and the
  prestaged SESSION composition (this stance + vref1 walk) showed a
  rise-segment FELL over_current — that harness likely carries the
  stale pre-tibia-150 height cfg (the documented copy-paste trap), so
  the stance-half promotion needs a corrected-cfg session-gate rerun
  pairing this checkpoint with `cw-dep-bcgait1-plant150-1` before the
  MEASURED-PLANT GATE BREAK's stance half can be declared closed.
  **DONE 08-22 (~19:4x): STANCE HALF CLOSED.** The corrected rerun
  (`eval_session`, rsifix stance x `cw-dep-bcgait1-plant150-1` walk,
  `--cfg-set actions.max_height_mm=137 bus.servo_params=loaded` — the
  run's own env-relevant cfg; goal./reward./train. keys are inert in
  the session harness) passes ALL THREE hard gates: no falls (the
  prestage's rise over_current FELL is GONE — it came from the default
  harness pairing `ppo_goal_cw_dep_vref1_r1` + stale default cfg, not
  from the checkpoint), rise z_end 170.3 mm, sit descends -57.8 mm.
  Soft: 5/7 PASS incl. fwd_heading yaw drift 8.8 deg (the axis
  plant150-1's own session narrowly missed at -10.6 with the old
  partner) and drive_height; only track_right/track_back miss — the
  documented all-model weakness (docstring baselines: back 12-34%,
  all models), NOT a tibia-150 regression. Strip watched: symmetric
  rise, level cruise, quiet hold, clean sit + re-stand. Evidence:
  `logs/ckpt_eval/rsifix_plant150pair_session_corrected/`. The
  tibia-150 download pair is now rsifix (stance) + plant150-1 (walk),
  both halves session-clean on hard gates.
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
- K_WALK_SWING LEVER CLOSED (08-22, both `cw-amp-m2-freeprog-term400-
  swing-{noamp,style05}` FAIL): same ~0.03m/15s statue-family
  ceiling as every prior arm (noamp/style05-v2/stylew2-v2/fixedcmd),
  gait_valid 3/6 and 5/6 det with intermittent sacrificed legs. The
  live signature was not the pre-cleared "single-leg farm" (all six
  legs register nonzero swing_count) but a close cousin of the
  bank's own pre-registered "shuffle" cheat: duty cycle heavily
  IMBALANCED across legs (one leg duty~0.99 almost never swings,
  another duty~0.02-0.19 almost never plants) — real strides that
  never organize into a coordinated tripod or net displacement.
  `env/reward_swing` stayed pinned ~0.05-0.06/tick the whole run in
  both arms (style added nothing on top, statistically identical to
  its noamp twin) — an order of magnitude below `walk_freeprog_pen`'s
  flat -1.4 to -1.5/tick floor, so the bonus never had the income to
  move a from-scratch PPO run out of the basin. ep_rew_mean fell
  every quarter both arms (genuine flat/declining FAIL, not
  continue-longer). This closes the WHOLE reward-side ladder for the
  M2 freeprog statue basin: term-penalty, std-anneal, staging, task-
  complexity, style-dose (0.5x-2x), and swing have now all failed
  the identical way (real, bank-verified/aligned mechanisms too
  small to out-bid the incumbent basin at 2M/from-scratch budget).
  FOLLOW-UP (first non-reward lever): `cw-amp-m2-freeprog-term400-
  rsi1-{noamp,style05b}` (`goal.walk_gait_start_frac=0.5`, RSI-for-
  walk mid-gait spawn) — changes the INITIAL STATE instead of the
  income, on the CURRENT freeprog+term400 pricing stack (the one
  prior RSI test, `cw-gait-rsi1` 08-11, predates that pricing
  entirely). If this also fails, the basin reads as INCOME-not-
  DISCOVERY and every accessible-gradient idea (dose or source) is
  closed for this family — next would be task restructuring or a
  short BC-pretrain phase, not another coefficient.
- RSI-FOR-WALK LEVER CLOSED, BOTH STACKS NOW TESTED (08-22, both
  `cw-amp-m2-freeprog-term400-rsi1-{noamp,style05b}` FAIL): mid-gait
  spawn (`goal.walk_gait_start_frac=0.5`) did NOT unlock sustained
  six-leg locomotion on the current freeprog+term400 pricing either
  — confirms `cw-gait-rsi1`'s 08-11 finding transfers to the new
  stack. Det fwd travel nominally rose (0.046-0.058m vs the
  ~0.03m statue ceiling) but this reads as an RSI RESET ARTIFACT,
  not real progress: `gait_valid` REGRESSED to 0/6 (worse than every
  reward-side arm in the family, which reached 3-5/6), with 2-3 legs
  consistently near-frozen (duty 0.9-0.99, swing_count as low as
  1-2) while the rest cycle — episodes coast on the RSI-seeded head
  start's already-moving state, then collapse into a partial-leg-
  drag pattern rather than sustaining the seeded gait. Style added
  nothing on top of RSI either (style05b statistically matches its
  noamp twin, if anything slightly worse; `amp/style_reward_mean`
  stayed low 0.03-0.09, no in-place-mimicry cheat). `ep_rew_mean`
  fell every quarter both arms, `env/reward_walk_freeprog_pen`
  pinned -1.5 to -1.7/tick — genuine flat-reward FAILs. CONCLUSION:
  the M2 freeprog basin is an INCOME problem, not a discovery-only
  one — changing WHERE episodes start does not help when nothing
  ever prices sustaining the gait once discovered. Every
  accessible-gradient idea tried for this family (term-penalty,
  std-anneal, staging, task-complexity, style-dose 0.5x-2x, swing,
  RSI) has now failed the same way. NEXT REAL LEVER (untested):
  task restructuring (e.g. a shorter/denser episode so the
  freeprog charge's ~500-step horizon stops dominating, or a
  reward that scores the WHOLE episode's net displacement rather
  than per-tick cross-track charges) or a short BC-pretrain phase
  on the motion library before RL (still consistent with the
  track's "demo is training data, not the controller" charter) —
  not another reward coefficient, gradient source, or reset-state
  tweak.
- CANDIDATE ROOT CAUSE FOR THE WHOLE M2 FREEPROG LADDER, TEXTUALLY
  GROUNDED, NOT YET TESTED (08-22, read while writing up the
  rsi1/swing FAILs — flagged for DIG-IN, no GPU spent on it yet):
  `AMP_LOCOMOTION.md` section 2 ("Why the Previous Program Stalled")
  describes the EXACT failure mode this whole 8-arm ladder keeps
  reproducing — "repeatedly converged to paddle-creep, dragging,
  narrow specialists, fragile choreography, or reward exploits" —
  and names the fix as a SIMPLE task reward (section 5.1:
  `r_linear_velocity = exp(-||v-v_cmd||^2/sigma_v)` +
  `r_yaw_velocity` + `r_upright` + a WEAK height regularizer, "a
  small number of interpretable reward families... avoid another
  large reward-term search") carrying a DOMINANT-enough AMP style
  weight (section 5.2: task/style in the 70/30-30/70 range, "strong
  enough that the policy cannot ignore it") plus MODEST physical
  regularizers (section 5.3, explicit: "Do not make stance slip the
  dominant reward or a hard early gate"). Every M2 freeprog arm to
  date (noamp/style05-v2/stylew2-v2/fixedcmd/swing-*/rsi1-*) instead
  reused the JOYSTICK track's SLIPWALK semantics-bank pricing stack
  wholesale: `k_walk_freeprog` (replaces the Gaussian kernel with a
  SHARP saturating linear score, cap 0.05 m/s — any cross-track
  jitter above 5 cm/s already maxes the charge), PLUS
  `k_loadslip_excess`, `k_drag_stance=8000`, `k_walk_idle_charge`,
  `walk_anchor_gate`, `walk_gait_gate` (a multiplicative income
  discount) — a large, harshly-saturating, UNGATED anti-slip
  apparatus built for a WARM-STARTED BC-refinement regime, not a
  from-scratch AMP actor. Measured signature across every arm:
  `env/reward_walk_freeprog_pen` (this stack's dominant single term)
  sits flat at -1.4 to -1.7/tick from step 0, ~15-30x
  `amp/style_reward_mean`'s realized income (0.03-0.09/tick even at
  2x style weight) — i.e. exactly the "task reward drowns out style"
  failure the brief's section 5.2 warns against, not a coincidence
  of dose. NOT YET TESTED: a clean-slate M2 arm implementing section
  5.1-5.4 close to literally (Gaussian velocity+yaw kernel, upright,
  weak height, modest action/torque/collision regularizers, NO
  freeprog/loadslip/drag-stance/idle-charge/anchor/gait-gate
  machinery, term_penalty kept per the suicide-economics finding
  which is a section-5.4-compatible reward choice not a termination-
  condition change) at task/style 50/50. This is DISTINCT from both
  prior reward families tried: the pre-freeprog "legacy" reward
  statued via a DIFFERENT exploit (frozen half-tripod overpaid by
  rise_finish/posture/height kernels, per the M2 -c1 finding) and
  freeprog statued via the shuffle/drag-basin family above — a
  brief-literal minimal reward has never been run. Needs careful
  design (which modest regularizers to keep so the OLD freeze cheat
  does not reopen) before training, i.e. real dig-in work, not a
  cfg toggle — flagged, not launched, this cycle.
- MEASUREMENT ARMS FOR THE REDESIGN, FIRST READING (08-22): before
  spending design effort on the section-5 minimal-reward rewrite,
  `cw-amp-m2-styleonly-v2` (amp-task-weight 0.0 / amp-style-weight
  1.0, teacher_v2, canonical pure-imitation AMP pretraining, NO task
  income or charges at all) directly tested whether the style
  gradient is learnable in isolation. VERDICT: FAIL, not a
  continuation, despite `ep_rew_mean` rising MONOTONICALLY every
  quarter (14.2/21.9/30.3/37.8 — the first-ever rising trend in the
  whole M2 freeprog family) and `amp/style_reward_mean` climbing
  0.06->0.119 (2x the pinned floor, discriminator healthy: d_real
  0.79/d_fake -0.96, unsaturated). DR-0 gate + fresh eval video both
  show the SAME frozen-tripod statue as every prior arm — gait_valid
  0/6 det AND sto, legs [1,3,5] (the mirror tripod of the swing/RSI
  family's [0,2,4]) sacrificed/frozen, speed ~0.006-0.025 m/s,
  prog_ratio ~0.00; the rising income bought a marginally more
  discriminator-plausible STATIC pose, not progress toward six-leg
  cycling — video overrides the reward-rising signal here (still
  short of the arm's own pre-registered 0.3 informative-pass bar
  too). READS AS: the AMP mechanism is confirmed alive (not
  saturated/dead) even at zero task competition, but too weak/slow
  on its own to organize six-leg coordination at 2M budget — rules
  OUT "just remove the task reward" as a one-line fix. Twin
  `cw-amp-m2-taskdown01-style1-v3` (task_weight 0.1, retry of a
  code-sync-REFUSED v2 that trained 0 steps) completed the joint
  read: FAIL, and the informative half — a task charge just 10% of
  the SLIPWALK stack ALREADY erases styleonly-v2's fragile gain
  (`amp/style_reward_mean` 0.087, LOWER than styleonly's 0.119;
  `ep_rew_mean` declining -2.4/-30.4/-69.2/-85.2, opposite of
  styleonly's rise) even though DR-0 gait_valid ticked up slightly
  (det 1/6, sto 3/6 vs styleonly's 0/6 both) — still the same
  near-static statue on video, legs [1,3,5] sacrificed. TASK/STYLE
  DOSE LADDER ON THE SLIPWALK-DERIVED REWARD IS NOW FULLY CLOSED
  (0.0/0.1/0.5/1.0/2.0 all tried, all FAIL, same failure family): no
  ratio of task-vs-style income on the EXISTING SLIPWALK apparatus
  works from scratch. This makes q_20260822T1815Z's diagnosis
  (the reward ARCHITECTURE itself, not its dose, is the wrong shape)
  a MEASURED finding, not just a textual reading of the brief — the
  next real M2 arm is the section-5 minimal-Gaussian-task reward
  rewrite (dropping freeprog/loadslip/drag-stance/idle-charge/
  anchor/gait-gate wholesale), not another task/style ratio point.
- MEASUREMENT ARMS CLOSED + SECTION-5 REWARD BUILT+BANK-TESTED (08-22):
  the +10M continuation `cw-amp-m2-styleonly-v2-c1b` VERDICTED FAIL —
  `amp/style_reward_mean` plateaued (quarter means
  0.10/0.20/0.16/0.15, never sustained above the 0.3 pass bar) but
  DR-0 gait_valid jumped to 5/6 det+sto (was 0/6 for the parent's
  frozen stork statue) with video showing genuine multi-leg cyclic
  stepping, not a static pose — a march-in-place/leg-twitch
  degenerate (fwd 0.02-0.03m/15s, slip 16-21/m): style organizes
  coordinated leg motion but has ZERO incentive to convert it into
  displacement, because the 60-dim discriminator feature set is
  body-frame-relative (a translating and a non-translating cyclic
  gait look identical to it absent slip) — an AMP-standard
  limitation, not a bug. This CLOSES the pure-style-alone
  measurement question definitively (paired with taskdown01-style1-
  v3's FAIL): task reward supplies "go somewhere", style supplies
  "look natural", neither alone suffices — exactly the brief's own
  design, now measured not just read. BUILT the section-5.1 literal
  minimal task reward as `AMP_MINIMAL_OVERRIDES`
  (`rl_move/tests/test_task_semantics.py`) with ZERO new production
  code: it is exactly the plain Gaussian velocity kernel + linear
  progress (`walk_task.py` `K_WALK`/`SIGMA_V`/`k_walk_prog`) that
  already fires whenever every SLIPWALK-only key
  (k_step_event/k_park_duty/k_walk_freeprog/walk_loadslip_gate/
  k_loadslip_excess/walk_gait_gate/k_walk_idle_charge/k_drag_stance)
  is left at its off default, plus `reward.term_penalty=400` kept
  (the real anti-suicide pricing fix, not SLIPWALK-specific).
  DELIBERATE CHOICE (per the taskdown01-v3 trace bullet above, option
  2): the legacy `env.compute_reward` posture/height/roll/pitch
  kernel was left ON, not zeroed — adopted deliberately as the
  brief's own upright/weak-height regularizer (5.1), eyes open, not
  by omission. Measured on the 3-seed scripted-twin bank: real travel
  beats every stationary twin (stall/park/stork/skate) by a modest
  ~230-250/ep margin (weaker than SLIPWALK's engineered 300+, as
  expected — no anti-park apparatus by design) and the stationary
  twins are bunched within ~15 points of each other (near-zero
  "trying beats refusing" gradient, pinned as a measurement, not
  patched — style is relied on to supply that gradient, which
  styleonly-v2/-c1b just proved it can). New tests 4/4 pass; full
  bank 163 pass / 4 skip / 1 xfail / 1 known-red (pre-existing,
  unrelated `fastprof`, untouched). CAVEAT CARRIED FORWARD, still
  untested: the taskdown01-v3 trace bullet's chronic
  height-tracking-failure confound (a trained policy sitting at
  50-86mm height error the whole run, independent of reward shape)
  is NOT ruled out by a scripted-twin bank (twins sit at the correct
  physical height by construction) — if the new arms below also show
  chronic `env/height_err_mm` >40mm, root-cause THAT before blaming
  reward shape again. LAUNCHED (this cycle) the brief's own
  pre-registered 3-arm task/style sweep on this reward:
  `cw-amp-m2-sec5-{task70,task50,task30}` (task/style 0.7/0.3,
  0.5/0.5, 0.3/0.7), from-scratch, teacher_v2 motion lib, same
  stress_mix command envelope as the retired freeprog family, 2M
  discovery each.
- TASKDOWN01-V3 PER-COMPONENT REWARD TRACE, A DESIGN CONSTRAINT FOR
  THE SECTION-5 REWRITE (08-22, read off `env/*` W&B history, no
  GPU spent): the legacy `reward.k_track=1.0` attitude/height
  Gaussian kernel (`env.compute_reward`'s `r_task`, the SAME
  mechanism the M2 -c1 dig-in blamed for the pre-freeprog frozen-
  statue exploit) stayed present, UN-ZEROED, in every freeprog-
  family arm including this one — but it is NOT acting as a standing
  bonus here: `env/reward_task` collapsed from 0.60 (reset pose) to
  a pinned 0.03-0.07/tick for the whole run, because
  `env/height_err_mm` sits at a chronic 50-86mm (sig_h=20mm ->
  the height factor inside the kernel is ~exp(-16) to exp(-8), i.e.
  near-zero) — the robot is NOT resting comfortably at the goal
  stance height; it is stuck low/crouched the entire run and never
  recovers, and the resulting `env/reward_height` quadratic charge
  (-0.7 to -0.28/tick) is the SECOND-largest per-tick penalty after
  `env/reward_walk_freeprog_pen` (-1.4 to -1.5/tick). So the ladder's
  statue is NOT currently being financed by an active k_track
  standing-still payoff (that channel is self-defeating here); it
  coexists with a large, un-diagnosed CHRONIC HEIGHT-TRACKING FAILURE
  that every arm in the ladder has been silently paying for and never
  fixed. Two concrete implications for whoever does the section-5
  redesign: (1) `reward.k_track`/`k_height`/`k_roll`/`k_pitch` (the
  legacy `env.compute_reward` terms) are NOT bit-exact-off in any M2
  freeprog arm to date — they still fire every walk tick unless
  explicitly zeroed, so "drop the SLIPWALK apparatus" must also
  either zero these or deliberately keep them as the brief's own
  upright/height regularizer (5.1) with eyes open, not by omission;
  (2) the chronic height error itself is worth a direct root-cause
  pass (is the goal height reference reachable from this policy's
  crouched stance, e.g. a walk-goal height sampled for a taller gait
  than the sprawled statue can reach, or a genuine height-tracking
  bug) BEFORE assuming the task-reward SHAPE alone explains the
  ladder — an untested confound the redesign should rule out first.
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
