# joystick - RL from the programmatic gait to joystick control

Last updated: 2026-08-22 (~18:5x: **THE DOSE LADDER IS SEED-SPECIFIC —
`stotight50-seed13` (best basin seed13 × -5.0) VERDICTED INFORMATIVE:
still passes the 60s DONE-gate (0 falls/48, gait 48/48, clean six-leg
video) but with WORSE margins than seed13's own -4.5 reading — slip
2.407→2.63, dir 36.4→38.21 (own-DR slip 2.763/dir 39.59); the
pre-registered PASS bar (slip ≤ 2.407) was missed and even the
INFORMATIVE ceiling (2.569) exceeded. Det did NOT soften (15s DR-0
det prog 0.79/slip 1.61, strong) and reward rose all run — reward and
gate agree; the -5.0 rung's gains are seed17-basin-specific, not
universal. **Champion candidate unchanged: seed13@-4.5 (slip
2.407/dir 36.4) keeps the fattest margins.** Follow-up batch
launched: `stotight50-seed23` / `stotight50-seed29` — does the deeper
floor transfer to the two seeds holding the named hardening gaps
(seed23 own-DR-alone dir 40.36; seed29 own-DR slip 2.736), or is the
ladder purely a seed17 phenomenon? Prior banner below.)

Previous entry (~18:4x: **LADDER STILL PAYING AT -5.5 —
`stotight55` (seed17, log-std-final -5.0→-5.5, final std ~0.004)
VERDICTED PASS on the 60s joystick session gate with the BEST
direction-following of any passer: slip 2.515 (parent 2.569), dir
34.97° (parent 38.02°, near the teacher floor ~35), own-DR pass slip
2.542/dir 33.7, 0 falls/48, gait 48/48, no sacrificed legs; det
improved yet again (15s DR-0 det prog 0.74/slip 1.63 vs parent
0.69/1.72) — five monotone rungs (-3.6/-4.0/-4.5/-5.0/-5.5) and the
det trade has never materialized past -4.5. Slip step is shrinking
(-0.102 → -0.054/rung) but the 3° dir jump is well outside the prior
rung's 0.6° step — not saturated. Follow-ups launched per the PASS
branch: `stotight60` (seed17, -6.0 — knee search continues) and
`stotight55-seed13` (best basin × this dose — champion-candidate
margins; seed13@-5.0 still training on train-4). Prior banner
below.)

Previous entry (~18:2x: **DEEPER NOISE FLOOR WIDENS EVERY
MARGIN — `stotight50` (seed17, log-std-final -4.5→-5.0) VERDICTED
PASS on the 60s joystick session gate: slip 2.569 (parent 2.671),
dir 38.02° (best of any passer), own-DR slip 2.623 (vs 2.736), 0
falls/48, gait 48/48; and the feared det-progress trade did NOT
appear — 15s det prog 0.69/slip 1.72 vs parent 0.65/1.83, det
IMPROVED. Dose ladder -3.6/-4.0/-4.5/-5.0 still monotone. Follow-ups
launched per the gate's own PASS branch: `stotight55` (seed17,
-5.5 — find the knee) and `stotight50-seed13` (best-basin seed ×
deeper dose — fattest-margin champion candidate, target slip <
seed13@-4.5's 2.407). Prior banner below.)

Previous entry (~18:1x: **DONE-GATE PASS REPRODUCES ON
EVERY TESTED SEED — recipe 4/4, seed-robustness question CLOSED.**
`stotight45-seed13` and `-seed29` both VERDICTED PASS on the full
60s randomized joystick session gate, completing the n=4 sample:
seed17 (original) slip 2.671 / dir 38.6; seed23 2.78 / 39.4; seed13
2.407 / 36.4 (widest margins of any passer); seed29 2.704 / 39.05.
All four: pass=true, 0 falls/48, gait_valid 48/48, no sacrificed
legs, videos watched det+sto both DR (clean upright six-leg gait);
training reward rose all run with std annealed to 0.011 on every
seed — reward and gate agree everywhere. Seeds 13 and 29 were the
lineage's two historically WORST basins at the -3.2 dose (1/4 pass
rate there) — the -4.5 noise floor converted a seed lottery into a
reproducible recipe; recipe property, not seed luck. The
seed-reproduction bar implied by q_20260822T1730Z is met; formal
gate-green + champion promotion remain operator-confirmed. Honest
residual margin gaps if the operator wants hardening: own-DR sto
margins (thinnest: seed29 slip 2.736/2.9, dir 39.4/40; seed23
own-DR-alone dir 40.36 a hair over on its own) and the 15s-rung det
progress trade (~0.85x clone) shared by the -4.5 passers. Prior
banner below.)

Previous entry (~17:5x: **THE DONE-GATE PASS REPRODUCES ON
A SECOND SEED** — `cw-dep-bcgait4-phasedir9-stotight45-seed23`
(identical stotight45 recipe, only seed 17→23) VERDICTED PASS on the
full 60s randomized joystick session gate: pass=true, 0 falls 48/48,
gait_valid 48/48, no sacrificed legs, slip 2.78 (cap 2.9), dir 39.4°
(allow 40); clean six-leg video at both DR scales. Margins thinner
than seed17 (own-DR-alone dir median 40.36°, a hair over the
allowance on its own; combined gate passes).)

Previous entry (~17:3x: **DONE GATE PASSED FOR THE FIRST
TIME** — `cw-dep-bcgait4-phasedir9-longrun17-stotight45` (fresh
reinit of the longrun17 recipe, only `--log-std-final` -3.2 -> -4.5,
final std 0.011) passes the full randomized 60s joystick session
gate: versioned evaluator pass=true, n=48 held-out episodes, ZERO
falls, gait_valid 48/48, no sacrificed legs, combined slip 2.671
(cap 2.9), dir_err 38.6deg (allow 40), and every mode individually
under caps (worst: own-DR sto slip 2.859). Sto slip fell
monotonically across the -3.6/-4.0/-4.5 dose grid (own-DR 15s-panel
3.00/2.87/2.48; siblings near-miss the session gate at 40.7deg dir /
2.94 slip). Videos watched (video-joygate rerun, det+sto, both DR):
clean upright six-leg alternating gait. Honest caveats: det softened
vs longrun17 (session det slip 2.30->2.55, dir 34.7->37.6, still
under caps; 15s rung prog 1.02x->0.85x clone) — the expected
noise-floor trade; own-DR sto margins thin. Checkpoint
`ppo_goal_cw_dep_bcgait4_phasedir9_longrun17_stotight45.zip` pulled
to controller, md5 9fb86d18 pod==controller. PROMOTION + formal
gate-green declaration flagged for operator confirmation
(OPERATOR_QUESTIONS q_20260822T1730Z); q_20260822T1520Z's assumed
answer (sto gap is a policy property fixable by dose) is CONFIRMED.
Same cycle: RAMP LEVER CLASS CLOSED — the 4-arm
allowramp2{slow,wide}-seed{23,29} grid verdicted 3x worse-on-both-
axes + 1 noise-edge nominal improver (slow29); no dose generalizes
across seeds; judgment closure, not the literal 0-for-4 trigger —
see Next item 3.)

Previous entry (08-22): (`-longrun17-cont1` VERDICTED FAIL on its
pre-registered prediction-if-false: the +4M continuation with std
held at 0.041 RETAINED longrun17's full det DONE-gate pass (det slip
2.38/2.65 vs cap 2.9, dir 34.5/37.6 vs 40, 0 falls 48/48) but sto
stayed out (slip 3.93 DR-0 / 4.36 own-DR, dir 50-51deg) while
training reward rose the whole run (91.6->225.6). CONTINUATION/
BUDGET LEVER CLOSED for the sto axis — sto is a reward/eval
divergence, not undertraining. Live probe: the stotight
log-std-final dose grid (-3.6/-4.0/-4.5, all finished, triage
pending); if sto slip stays ~4 across doses, next is train-time
sto-robustness (perturbation training or sto-aware pricing). Also
settled this day: recipe seed pass rate 1/4 (seed17 only;
13/23/29 FAIL) — "seed lottery too thin to farm"; allowramp
generalization REFUTED on seeds 23+29.)

Previous entry (08-22, operator-ordered FORMAL SESSION-GATE
reading on `-longrun17`, after the operator live-accepted the
checkpoint under real joystick input on the Mac viewer: the full 60s
randomized DONE gate (`eval_joystick_gate.py`, held-out stress_mix,
n=12 det+sto at DR-0 AND own-DR 0.35) is **FAIL overall but
DET-ONLY PASSES EVERY AXIS at both DR scales** — zero falls 48/48,
gait_valid 48/48, det slip 2.30 (cap 2.9), det dir_err 34.7deg DR-0
/ 37.4deg own-DR (allow 40). The sto half alone fails it (slip
4.0, dir 51-52deg -> combined medians 3.325/45.7). The
operator-live-vs-gate delta is RESOLVED as det-vs-sto, NOT command
distribution: the det policy follows held-out reverses/stops/turns
it never trained on (trained forward-only fixed 0.08!) and beats the
phase clone's own session reading ~5x on slip (15.9->3.3) and ~20deg
on direction. PROMOTION NOT EXECUTED (operator conditional was
pass-gated); sto-calibration question filed
(OPERATOR_QUESTIONS q_20260822T1520Z, assumed answer: gate stands,
sto gap is a policy property -> sto-robustness arm after
longrun23/29 settle the seed pass rate). Presentation caveat
(fb_20260822T145428): the phasedir lineage deliberately trains with
the legacy `bc_anchor_knee_abs=1.0` dialect — do not present it as a
clean current-convention imitation line. Artifacts:
`logs/ckpt_eval/longrun17_joystick_session_gate_v1/`.

Previous entry (08-22, CORRECTION to the entry below it:
`phasedir9-longrun17`'s FAIL verdict was written before its own eval
finished syncing — a premature-verdict race, not a real reading.
Recomputed from the synced report + this run's own W&B summary
(triple-confirmed), `-longrun17` (seed17) is actually a **DET-mode
PASS** of the rung-A clone-relative gate — the first in 34+ arms —
at both DR-0 (progress 1.02x clone, slip 0.74x, speed 0.069 m/s) and
its own DR-0.35 (0.94x/1.01x/0.067, thinner). `-longrun13` (seed13)
stays correctly verdicted FAIL/worse (0.792x/1.286x). So the "budget
lever is closed end-to-end" conclusion below is WRONG for seed17 and
right for seed13 — a genuine seed-dependent divergence on an
identical recipe, not a settled answer either way. Ledger set to
`PASS (partial)` pending: reproduce `-longrun17` independently before
any promotion (`-longrun29` seed29 RUNNING), and root-cause the
seed13/seed17 divergence. **BC-anchor/phase-lock family-boundary
DIG-IN RESOLVED same day** (per-tick trace,
`rl_move/sim/trace_bc_cadence.py`,
`logs/ckpt_eval/pd9seed17_bc_cadence_trace`): NO cadence gap exists —
policy, clone AND raw teacher all cycle at 0.76 s == bc_target ==
the 0.75 s phase clock (TripodGait.period=0.75 == 1/walk_phase_hz by
construction). The "swing_s_mean ~30% slower" premise was a
contact-segmentation ARTIFACT: the clone's 0.25 s "swing" is double
lift-offs per cycle (24-36 lifts/leg/15 s vs ~19 cycles, lift-to-
lift 0.44 s vs 0.76 s) splitting its swings; the policy single-
swings cleanly at 0.372 s vs the raw teacher's realized 0.345 s
(+8%). Supervision is honest in the eval regime (policy-vs-bc_target
MSE 0.00136; the clone's own is 0.00549; xcorr ~1.00 at the same
~3-tick servo lag). LEVER CLASS CLOSED: anchor dose, walk_phase_hz,
phase-lock plumbing all exonerated. The seed divergence is therefore
NOT a supervision defect — it is the known init/seed-basin lottery
on a reward surface that is ~flat across honest and drag basins at
annealed-low std; the residual rung deficit on failing seeds is
loaded-foot slip during stance at MATCHED gait timing/stride/duty
(seed17 slip 2.85/m vs clone 1.89). Next budget = measure the
recipe's seed pass rate (longrun29 + longrun23), then promote the
best det passer. Superseded text below kept for the lineage record;
treat its "FORK RESOLVED/EXONERATED end to end" language as void.

Previous entry (08-22, phasedir9-seed17 VERDICTED FAIL-as-
reproduction): pd9-stdanneal's 2M near-pass (0.873x progress, 1.08x
slip) did NOT reproduce on a second seed. FOLLOW-UP `phasedir9-
longrun13`/`-longrun17` (fresh re-inits, NOT continuations — same
stacks/seeds, --steps 2M->4M, anneal still ending ~1.2M so ~2.8M
steps run converged vs ~800k): longrun13 (from the GOOD seed) got
WORSE (progress 0.873x->0.792x clone, slip 1.08x->1.286x); longrun17
(from the BAD seed) — SEE CORRECTION ABOVE, this is now a PASS
(partial), not "stayed FLAT". Zero falls, gait_valid 6/6, clean video
on all four runs (stdanneal/seed17/longrun13/longrun17). Condensed
08-22 for the <=120-line budget — full lineage detail lives in
`RL_LOG.md` + ledger verdicts, not here. Keep this a short screenful:
Goal / Now / Next.

## Goal

Start from the simple programmatic gait (the scripted tripod teacher
and its BC clones) and use RL to make it genuinely joystick
controllable in sim.

**DONE gate (pre-registered, operator 08-21):** one policy (or the
session-controller stack) follows a randomized 60-second joystick
command script in MuJoCo — direction changes, stops, reverses, turns —
with:

- ZERO falls across the full panel (n>=12 episodes, det+sto, DR-0 and
  the run's own DR, held-out command seeds);
- directions actually followed (heading obedience judged against the
  teacher clone's measured ~35 deg tick-level stride-sway floor —
  compare deltas, not raw values);
- little slip: slip/m no worse than the scripted teacher's measured
  band at the calibrated plant (<= ~2.9; teacher band 1.4-2.9).

## Now (inherited state, 08-21)

- Scripted tripod teacher verified clean at the measured tibia-150
  plant: 0.06-0.10 m/s x 4 headings, zero falls, slip/m 1.4-2.9,
  full fast servo profile.
- Best starting checkpoint: the phase-conditioned BC clone
  `ppo_goal_cw_bcgait_init_fullprof_phase1` (holdout act err 0.0040)
  passes the entire direction-first curriculum with ZERO RL — all
  fixed headings incl. rear, irregular heading changes, stops.
- Fallback baseline: the download hierarchy
  (`footlow2_hard1` stance + `bcgait1_hard1` walk + session
  controller; held-out session gate det 0.967 / sto 0.853, n=600).
  Note: pre-08-22 checkpoints trained on the old 128 mm plant.
- Hard-won evidence: five fast-gait RL levers failed AS RUN because
  the reward was not aligned with the eval. Per the 08-21 ruling
  those are MISALIGNMENT results, not dead ends. `phasedir1` is
  additionally ENV-CONFOUNDED (convention-corrupted sim).
- **phasedir2-8 lineage (full detail: RL_LOG + ledger verdicts;
  every arm zero falls, gait 6/6, clean video)**: eight consecutive
  FAILs of the aligned-reward stack on the clone-relative rung-A
  gate, each refuting one lever class in turn — staged curriculum
  (obedient but slow, 0.836x progress), loadslip reprice (slip
  unpriced 1.41x), ent-coef anneal (std barely moved), warm-log-std
  override (progress PASS 0.984x but SLIP-FINANCED — drag family,
  dig-in-pinned), band retighten (VALUE lever refuted), k_drag_stance
  8000/4000 (STEP FUNCTION, identical slow optimum both doses), and
  finally phasedir8's stride-EMA kernel (allow 24, `walk_kernel_vel_ema`)
  which still missed (prog 0.770x, best of lineage to that point) —
  dig-in found the det/DR-0 pricing calibration DOES NOT TRANSFER to
  the noisy optimization regime (no separating allowance exists
  between the honest noisy tail and the det drag cheat under PPO
  exploration). REPAIRED via `train_ppo_mjx --log-std-final/
  --log-std-anneal-frac` (forced noise-anneal so std converges to
  the det regime where pricing IS measured-aligned) +
  `reward.walk_course_overspeed_ref_floor_m_s` (ramp-drift insurance).
  Bank 34/34.
- **phasedir9 (anneal from raw BC clone) vs phasedir9b (anneal
  continuing the pd8 cheat-committed checkpoint), same stack+seed**:
  `-9` UNDERTRAINED/near-pass at 2M (zero falls 24/24, gait 6/6,
  slip/dir_err/speed all inside gate, progress 0.873x clone — best
  of lineage, narrow miss of 0.9x cap, reward+drag-charge still
  moving). `-9b` VERDICTED FAIL: WORSE than pd8 itself on every axis
  (prog 0.704x, slip 1.506x, speed 0.054) despite drag charge
  falling 4x — dodged the bill by shrinking per-stance travel, not
  slipping less (walking in place). ROOT CAUSE (dig-in resolved):
  INIT-BASIN SELECTION, not checkpoint recency — `-9b`'s final
  TRAINING reward is equal-or-better than `-9`'s, so this reward
  stack's optimization-regime surface is ~FLAT across the honest and
  drag basins at annealed-low std; PPO is a local polisher and INIT
  decides which basin gets polished. **LINEAGE RULE (binding for this
  reward stack): pricing/regime repairs re-init from a pre-cheat
  checkpoint (teacher/clone), never continue-train an
  already-converged one.**
- **phasedir9-cont1 VERDICTED FAIL as a continuation** (+4M steps
  from the `-9` checkpoint): regressed hard on every clone-relative
  axis (progress 0.873x->0.66-0.71x, slip 1.08x->1.6-1.7x, speed
  dropped below the 0.06 floor), zero falls/gait-6-6 preserved. W&B
  ep_rew crashed 27->-670 by ~1.4M then only partly recovered to
  152-213 by 4M — collapse-then-partial-recovery, not steady
  climbing. Same mechanism as `-9b`, opposite direction: a GOOD init
  drifted into a WORSE basin because the reward surface can't tell
  them apart at annealed-low std. `-9` itself is unaffected, still
  the lineage's best 2M reading.
- **phasedir9-seed17 VERDICTED FAIL-as-reproduction** (08-22): the
  reproducibility replicate landed at/below pd8's own level (progress
  0.727x clone vs pd9's 0.873x and pd8's 0.766x; slip 1.27x vs pd9's
  1.08x and pd8's 1.254x), matching the pre-registered
  prediction-if-false almost exactly ("lands far below pd9... back
  near pd8 or worse"). Zero falls/gait-6-6 preserved both seeds.
  READS AS: pd9's near-pass was partly seed luck riding the
  log-std-anneal fix, not a reliably repeatable recipe — do not
  trust a single good seed's numbers as the lineage's ceiling.
  TELEMETRY LEAD for the dig-in this decides (see banner): W&B
  `train/bc_anchor_loss_walk` is already tiny (0.00005-0.0007) and
  `env/walk_anchor_frac` already high (0.80-0.93) on BOTH seed13 and
  seed17 — the phase-locked BC-anchor aux loss reads as converged —
  yet BOTH seeds' realized `swing_s_mean` runs ~30% slower than the
  teacher/clone (0.34-0.36s vs 0.25-0.27s). A near-zero action-space
  anchor loss coexisting with a large realized-cadence gap points at
  stochastic-action/servo-slew/plant realization as the boundary, not
  supervision strength — raising `train.bc_anchor_coef` blind is
  unlikely to help and was NOT attempted. LAUNCHED (fresh re-init
  from the raw BC clone, NOT a continuation — the lineage rule is
  intact) `phasedir9-longrun13`/`-longrun17`: same stacks/seeds as
  stdanneal/seed17, only `--steps` 2M->4M and
  `--log-std-anneal-frac` 0.6->0.3 (anneal still ends at the same
  absolute ~1.2M step; ~2.8M steps in the converged regime instead of
  ~800k) — tests the 08-21 "needs to go longer" branch on both seeds
  before committing to the phase-lock dig-in's on-pod tracing work.
- **phasedir9-longrun13 VERDICTED FAIL, -longrun17 CORRECTED TO
  PASS (partial)** (08-22, same cycle — this pipeline trains 2-4M
  steps in single-digit minutes, so both finished before triage did;
  `-longrun17`'s first verdict was a premature-verdict race, written
  before its own eval synced — see the top-of-file correction
  banner): the budget lever is NOT uniformly closed. `longrun13`
  (from the seed that near-passed at 2M) got WORSE with 2x the budget
  (progress 0.873x->0.792x clone, slip 1.08x->1.286x, confirmed).
  `longrun17` (from the seed that missed at 2M) actually CROSSED the
  rung-A gate at 4M — DR-0 det progress 1.02x clone, slip 0.74x,
  speed 0.069 m/s, zero falls, gait_valid 6/6, clean video — the
  first full det-mode PASS in the entire phasedir1-9/longrun lineage.
  Both runs' W&B ep_rew_mean rose strongly through the extra budget
  (quarters ending +187/+193 vs either 2M run's ~-300), but only one
  seed's gate metrics tracked that rise — a seed-dependent basin
  outcome, not a clean reward<->eval divergence story for both. Zero
  falls, gait_valid 6/6, clean 6-leg video, all four runs (stdanneal/
  seed17/longrun13/longrun17). Do not respec a 3rd continuation off
  any converged phasedir9 checkpoint, and do not retry another
  anneal-frac/--steps combination blind — DIG-IN queued first: (a)
  reproduce longrun17 independently, (b) root-cause the seed13-vs-
  seed17 divergence, alongside the on-pod per-tick
  trace (bc_target cadence vs realized policy cadence vs raw
  un-phase-locked teacher cadence) before any anchor-dose/phase_hz
  reward edit.

## Next

1. **CLOSED 08-22, all 3 remaining rise-bank items, root-cause (not
   re-measurement)** (7/7 tibia-150 residue now closed except
   fastprof, a separate already-tracked item): PLANT_SPEC's
   height-window "failure" was never the window — it was
   `goal.rise_height_mm=[108,114]`/`actions.max_height_mm=115`, the
   PRE-tibia-150 (128 mm) belly->plant height target, never updated
   when the tibia grew ~22 mm; the demonstrated reference
   deterministically settles at h_rel=131.94 mm (all seeds), ~24 mm
   past the stale target, tripping only `height_ok` while every other
   PLANT_SPEC check passed. Recalibrated the target to `[128,136]`/
   `137` (brackets the measured settled height, `RISE_OVERRIDES`/
   `SCORE_OVERRIDES` in `test_task_semantics.py` only — `LOWER_
   OVERRIDES` and other rise_height_mm call sites weren't broken,
   left untouched). `getup_honest_ordering`'s partial-crouch pricing:
   the one-shot progress ratchet (`reward.getup_k_progress`) didn't
   clear the honest rise motion's own extra regularizer cost over
   freezing (partial -12.16 < freeze -11.26 at k=60); recalibrated
   60->200 (partial +10.8 > freeze -11.5, full GETUP-bank ordering
   preserved, swept 60-500). Bank now 152 pass / 1 known-red
   (fastprof) / 4 skip / 1 xfail. The circular blocker is UNBLOCKED
   on the bank side: `extract_rise_ref.py --blend-mode ik`
   (built+tested 08-22) can remint a compliant reference once a
   tibia-150 stance source checkpoint exists, and a tibia-150 stance
   retrain arm can now be spec'd+launched (bank no longer red) —
   still gated only on joystick/amp GPU-budget priority, not on any
   remaining test. `cw-stand-footlow2-plant150-1` (train-1, warm-start
   from `ppo_goal_cw_stand_footlow2_hard1`, byte-identical recipe,
   only the plant now tibia-150) VERDICTED FAIL 08-22 — but HALF
   CONFIG BUG: rise/det 1/6, rise/sto 0/6 (parent 5/6 det, 6/6 sto),
   yet the launch's copy-pasted `--cfg-set` list carried the
   PRE-tibia-150 `actions.max_height_mm=115`/
   `goal.rise_height_mm=[108,114]` unchanged, never picking up the
   SAME 08-22 recalibration (`[128,136]`/137) already used by
   `test_task_semantics.py`'s bank — a gap in that fix's own scoping
   note (see CURRENT_TRUTHS). Re-evaluating the frozen checkpoint with
   the corrected cfg (zero retraining) turns bridge/crouch/flat rise
   CLEAN (1.4-3.9mm err, det 1/6->3/6, sto 0/6->4/6) — the policy had
   been overshooting the stale target by the tibia-150 height delta.
   Genuine residual, unmoved by the cfg fix: RSI-reset (mid-ramp
   DeepMimic spawn) starts fail every episode (5/5, ~22-29mm
   undershoot, more roll wobble) — passed fine at the 128mm parent
   (4/5), so this is real and tibia-150-specific. hold/lower unaffected
   (6/6 both). **LAUNCHED 08-22**: `cw-stand-footlow2-plant150-2c-
   heightfix` (train-5, same checkpoint, ONLY the corrected cfg, 10M
   more steps) — tests whether removing the reward-target fight also
   helps the harder RSI case; still targets the session gate's
   tibia-150 sit-fall (the stance half of the MEASURED-PLANT GATE
   BREAK, still open) so the download pair's stance half can be
   re-promoted alongside the walk half. **VERDICTED FAIL 08-22,
   ROOT-CAUSED**: RSI stayed pinned at 22-29mm height error through
   the full 10M extra steps (det rsi 0/3, sto rsi 0/2, zero movement
   from the pre-training baseline); bridge/crouch/flat held steady
   (det 3/6, sto 4/6, no regression). Training reward flattened after
   Q1 (120.6/201.1/198.1/200.6) — reward-flat + eval-flat, a genuine
   stuck mechanism per the 08-21 ruling, exactly the gate's own
   pre-registered FAIL branch. ROOT CAUSE found (not just
   re-confirmed): `sim_env.py`'s RSI height-schedule rewrite anchored
   the episode's ABSOLUTE height target to `rise_ref_belly2plant.npz`'s
   OWN recorded final height (`ref["h"][-1]` = 110.96mm) — that npz
   predates the tibia-150 change; the SAME `q_rad` trajectory settles
   at 131.94mm on the current sim (the exact number the
   `rise_valid_plant` fix already measured for a different bug). The
   ~21mm gap matches the observed RSI error almost exactly: every RSI
   episode was being TRAINED toward a target ~21mm below the real
   `goal.rise_height_mm=[128,136]` window the eval actually grades
   against. FIXED: the RSI schedule now anchors to the episode's own
   live-cfg target (already correct post-tibia-150) and uses the
   reference array only for FRACTIONAL progress at the spawn point —
   robust to the stale absolute scale. 2 new regression tests
   (`test_rise_rsi_height_target.py`, both pass) lock the fix; full
   `test_task_semantics.py` bank re-run clean (159 pass, only the
   pre-existing known-red `fastprof` fails, unrelated). **LAUNCHED
   08-22**: `cw-stand-footlow2-plant150-3-rsifix` (train-5, same
   checkpoint, ONLY the RSI mechanism fix, 10M more steps) — tests
   whether RSI now actually improves once its own reward target
   agrees with the eval target.
2. **Evaluator half DONE 08-22** (`rl_move/sim/eval_joystick_gate.py`
   + `test_eval_joystick_gate.py`, 8/8 pure-aggregation tests, no sim
   touched): a reusable, versioned 60 s randomized joystick-session
   gate — pure orchestration around `eval_checkpoint.py` with ONE
   pinned held-out command bundle (`goal.walk_cmd_mode=stress_mix`
   [full family: random_hold, flip_180=reverse, sweep_circle/
   square=turns, stop_go, jitter] + `walk_cmd_resample_s=4.0` +/-
   jitter 0.5, `--episode-seconds 60`, seed base 90000 — a range no
   other harness/training run draws from), run at DR-0 + the
   checkpoint's own DR, det+sto, aggregated into ONE PASS/FAIL against
   the DONE gate's own numbers (zero falls, slip/m <= 2.9, dir_err
   median within 5deg of the teacher's 35deg floor). FIRST-EVER
   READING (n=12x2 det/sto, DR-0, `logs/ckpt_eval/
   phase1_clone_joystick_gate_v1/`): the phase clone
   (`ppo_goal_cw_bcgait_init_fullprof_phase1`) — previously verified
   only against FIXED heading-per-episode panels — FAILS the real
   randomized multi-segment session hard: zero falls / gait_valid
   24/24 (the gait itself never breaks), but slip/m med 15.9 (cap
   2.9) and dir_err med 65.4deg (allow 40deg) once commands actually
   change direction/stop/reverse mid-episode instead of holding one
   heading. This is the first measured, reusable confirmation that
   "passes the direction-first curriculum" (fixed headings) and
   "joystick controllable" (this DONE gate) are different bars — the
   gap RL fine-tuning (Next item 4) must close. **DONE 08-22**: the
   two follow-ups flagged here (per-leg duty/swing/sacrificed-frac
   aggregation across the whole panel, and an opt-in `--video` flag
   replacing the hardcoded `--no-video`) are landed and tested
   (`test_eval_joystick_gate.py` 11/11, +3 new per-leg tests incl. a
   regression test for the exact frozen-tripod pattern seen this
   cycle on the AMP track). Semantics-bank half NOT started
   dedicated to this exact 60s session (the walk task's own reward is
   already alignment-tested per-mechanism via `test_phasedir_
   semantics.py`, 34/34); a session-specific bank is lower priority
   until a candidate gets close enough to the gate numbers to need
   cheat-proofing at this exact horizon.
3. **CLOSED 08-22**: the loadslip-band/drag-stance-dose/allowance
   lever family AND the "anneal noise, continue training" lever are
   both measured-refuted for this lineage (see Now: phasedir6/7/7b,
   and the `-9`/`-9b`/`-9-cont1` init-basin-flatness finding).
   **FORK CORRECTED 08-22 (was wrongly marked closed for ~15 min by
   a race)**: `phasedir9-longrun13`'s ledger verdict (seed13, WORSE:
   0.792x/1.286x) is correct and confirmed. But `-longrun17`'s FIRST
   committed verdict (~0.72x/1.32x, FAIL) was written at 14:03:08,
   BEFORE its own eval finished syncing (gate report synced 14:03:45,
   owncfg 14:04:04) — a genuine premature-verdict race, not a
   judgment call; a second concurrent-cycle sync at 14:07:02
   inherited the same stale numbers without re-checking the by-then-
   available real data. CORRECTED (triple-confirmed: on-disk
   report.json, `ops.sh report`, and this run's own W&B
   `eval/dr0/walk_det/*` summary all agree): `-longrun17` (seed17) is
   a **DET-mode PASS** of this lineage's rung-A clone-relative
   gate — the first in 34+ arms — at BOTH DR-0 (progress 1.02x clone,
   slip 0.74x, speed 0.069 m/s, all caps cleared) and its own DR-0.35
   (0.94x/1.01x/0.067, thinner margin). Zero falls, gait_valid 6/6,
   clean roll, no sacrificed legs, video shows a clean 6-leg
   alternating gait. STO still fails (as it always has lineage-wide —
   the clone's own sto baseline is itself degenerate, so sto was
   never part of the ratio criteria). Ledger status set to `PASS
   (partial)`, not a closing PASS: own-DR margin is thin, sto is
   still bad, and — the real puzzle — seed13 and seed17 ran the
   IDENTICAL recipe/steps/anneal and landed on opposite outcomes
   (worse vs. first-ever pass). That seed-dependent divergence, not a
   uniform "budget helps" or "budget doesn't help" story, is now the
   open question. DIG-IN state: (a) reproduce `-longrun17`'s reading
   — `-longrun29` (seed29) RUNNING, `-longrun23` (seed23) queued this
   cycle: with seed13 FAIL + seed17 PASS the question is the recipe's
   PASS RATE, n=4 seeds decides promotion strategy (select-best-of-N
   is legitimate if the surface is a seed lottery). (b) the family-
   boundary trace is **RESOLVED 08-22** (see banner): no cadence gap,
   supervision exonerated, `swing_s_mean` premise was clone contact-
   chatter; the divergence is the init/seed-basin lottery, and the
   failing seeds' deficit is stance slip at matched gait timing. Do
   NOT spend arms on anchor dose / walk_phase_hz / phase-lock edits.
   **UPDATE 08-22: DIG-IN item (a) is DONE — n=4 seed sample
   complete.** `longrun23` (0.818x progress/1.175x slip clone) and
   `longrun29` (0.740x/1.296x, worst of 4) both verdicted FAIL,
   landing in the same regression basin as seed13 — pass rate 1/4
   (`longrun17` only). "More seeds" is closed as a lever; per-seed
   promotion is legitimate (operator already live-accepted
   `longrun17` — see the file banner) but does not fix the recipe.
   Follow-up on the pricing side (item (b)'s "stance slip" lead):
   built+bank-tested `reward.drag_stance_allow_ramp_steps`/`_mm`
   (`test_drag_allow_ramp.py` 6/6) to directly re-attack the
   regime-gap (a fixed 24mm det-calibrated drag allowance overtaxes
   honest noisy exploration before the log-std anneal converges) by
   loosening the allowance early (48mm) and annealing it to the same
   24mm target in lockstep with the noise anneal. Single-change A/B
   on all 3 FAILing seeds (13/23/29, run in parallel by concurrent
   cycles): only **seed13** (`phasedir10-allowramp-a`) improved,
   moving BOTH clone-relative axes toward the gate at once for the
   first time in the whole lineage (progress 0.792x->0.830x, slip
   1.284x->1.162x) — still a FAIL (short of 0.9x/1.15x) but the first
   lever ever to help both axes together. **seed23** and **seed29**
   both got WORSE on both axes (seed23 0.818x/1.175x->0.792x/1.217x;
   seed29 0.740x/1.296x->0.725x/1.466x) — 1/3, FAIL, refuting it as a
   general fix. Zero falls/gait 6/6/clean video, no pathology, all
   three arms — a real basin effect, not a regression. Reads as: the
   regime-gap diagnosis is real, but a fixed ramp schedule is itself
   subject to the same per-seed basin lottery as every other lever
   here. Mechanism/code stays banked; do not arm it as a default off
   seed13 alone. **RAMP LEVER CLASS CLOSED 08-22 (~17:1x, judgment
   closure)**: the 4-arm grid (`allowramp2slow-seed23/-seed29`
   ramp_steps 1.2M->2.4M; `allowramp2wide-seed23/-seed29-b` ramp_mm
   48->64) came back 3 worse-on-both-axes (wide23 0.65x/1.65x,
   wide29 0.62x/1.74x, slow23 0.65x/1.70x — each worse than BOTH its
   no-ramp and 48mm baselines) and 1 nominal improver (slow29
   0.78x/1.25x vs 0.740x/1.296x and 0.725x/1.466x — prediction-if-
   true technically fired, so the strict 0-for-4 auto-closure did
   NOT literally trigger; recorded honestly). But the improver's
   prog delta (+0.044x) is at the edge of 6-ep noise, its slip delta
   inside noise, and across 6 ramp arms x 3 seeds every dose helps
   exactly one seed and hurts the rest (48mm->seed13 only,
   slow->seed29 only, wide->nobody) — a per-seed basin lottery, not
   a dose-response, with the best reading anywhere (seed13 0.830x)
   still far from the 0.9x gate. Zero falls / gait 6/6 / clean video
   all 4 arms; reward rose or peaked-then-tightened in all 4 while
   gate metrics mostly fell (misaligned-surface signature, not
   undertraining). CLASS CLOSED for failing seeds by judgment on
   this evidence; redirect stands: sto gap first (stotight dose
   grid — evals manually launched 17:0x after the 16:18 manual
   FINISHED-flip bypassed the watcher's finish-detection prestage;
   gate+joygate artifacts land in `logs/ckpt_eval/
   cw_dep_bcgait4_phasedir9_longrun17_stotight{,40,45}_*` — TRIAGE
   THEM NEXT CYCLE, no auto-spawn will fire for already-FINISHED
   runs), then item (b)'s matched-timing stance-slip mechanism
   (seed17 2.85/m vs clone 1.89/m).
4. RL fine-tune from the phase clone (and a walk-champion arm as
   control) with the reward aligned to the gate metrics, resuming
   the staged heading curriculum; extend budget while reward and
   gate metrics rise together.
5. Widen the command distribution toward the full joystick envelope
   (speeds, yaw, strafe, stops) and DR-harden to own-DR zero-fall.

## Rules of the road

- Reward rising + gate metrics bad = realign reward with the gate
  and/or continue longer — never a one-line FAIL
  (`RUN_INTERPRETATION_RULES.md`).
- Do not park on operator input; assume-and-go with a recorded
  assumption. Physical-robot items are the only true waits.
