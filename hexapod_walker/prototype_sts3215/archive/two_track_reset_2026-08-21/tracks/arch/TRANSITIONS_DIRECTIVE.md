# Operator directive (2026-08-13 ~21:00 UTC): ONE model for operator-commanded rise → walk → sit → rise cycles

Two jobs for the orchestrator, in order. Both target the same end
state: a single policy that follows operator mode commands (rise,
walk, sit/lower, hold) and executes every transition without falling.
CODE items land first (default-off, bit-exact, tested — house rule),
then the arms run. This doc is the spec of record; arch/STATUS.md
"Next" points here.

## What already exists (do not rebuild)

- **Specialist composition PASSES with zero falls** (08-11):
  `eval_handoff.py` — rise→settled-hold→walk-champion switch-in on the
  specialist's exact physical state, 12/12 zero falls, air AND loaded
  servos, the scripted 1.5 s blend adds nothing; `eval_handoff_reverse.py`
  — walk→stop→lower, zero falls. The full sim joystick cycle composes
  across TWO models with eval-side frame re-anchoring. Those two
  scripts contain the proven **re-anchor mechanics** (goal refs/height
  anchor re-based on the current settled state, qpos/qvel/ctrl and
  slew state carried physically) that both jobs below reuse.
- **Dual-core mode-gated GRU** (`DualGruActorCriticPolicy`,
  `--gru-dual`) is the proven single-net architecture: dual1 (10M)
  walks 6/6 with zero sacrificed legs while hold/lower stay 6/6.
- **DAgger redistillation works** (08-13, `distill_gru.py`):
  `ppo_goal_cw_gru_dual_bc_dagger1.zip` put rise into the BC init for
  the first time (n=12 det 3/12 with non-crouch wins). `cw-arch-gru-dual2`
  (backlog) tests whether RL+anchors lift it; its result feeds the
  warm-start order below.
- **Mode is currently a once-per-reset draw.** `GoalGenerator.sample`
  precomputes the whole ref trajectory; `_z0` (height anchor),
  `_h_target`, `_is_rise`/BC-anchor eligibility are frozen at reset.
  The obs mode one-hot is re-derived per tick from `_goal_traj.mode`,
  and most reward gates already check mode per tick — the machinery is
  CLOSER to sequence-ready than it looks, but mid-episode switches
  need the CODE below.

## Failure ledger this directive is designed around

Every design choice below traces to a measured failure. Do not
re-litigate these; cite them.

1. **Shared-trunk interference** (anchor1/2/3): stance-tick anchoring
   through one recurrent trunk froze walking — three levers refuted.
   → Both arms use the dual-core GRU. Never a shared trunk.
2. **Walk-tick anchoring freezes walk** (gaitbc1, transbc1, anchor1,
   twice-closed +1 GRU reproduction). → `train.bc_anchor_walk=0`
   always; stance anchors ON (they protected stance in every arm).
3. **Rise BC-demo poverty** (ft1/ft2, hfloor1): RL never invents a
   skill with zero demos; supervision-aim levers don't fix data
   poverty. → Warm-start order below; transition demos include rise
   segments from every start kind.
4. **The two stances DIFFER** (~72 mm crouch-stand vs ~142 mm plant;
   play.py bridge): naive policy handoff collapses into a belly
   shuffle. → Transitions must be in the training/demo distribution,
   and refs must re-anchor at each switch, not at episode start.
5. **Height refs are start-relative** (`_z0` frozen at reset;
   belly-start vs plant-start episodes behave differently, +45/−60 mm).
   → The mode-seq feature MUST re-anchor `_z0`/href at each segment
   switch or rise-after-lower aims at a stale frame. This is the #1
   hidden-state trap in the whole directive.
6. **Engagement snap** (08-13 takeoff audit, 27 tapes): at a control
   handoff the policy requests a ~100° whole-body reconfiguration on
   tick 0 and rides the slew clamp; roll crosses 5° at median 0.88 s.
   Mode switches inside one policy are the same hazard class. → v1
   ships a segment blend window (cfg, like `goal.walk_cmd_blend_s_*`);
   a switch-window tilt charge is the PRE-REGISTERED follow-up knob,
   not in v1 (one variable at a time).
7. **Multitask lesson** (mt-a/b/c at 2M and 20M): training a wide
   command set from scratch → crouch-splay creep; bolting new commands
   onto a walking prior → prior survives, new skills not acquired.
   → Warm from a full-skill init and make TRANSITIONS the only new
   thing. Do not widen anything else in the same arm.
8. **Context mismatch** (bc2): stance demos at 10 s regressed on the
   15 s eval context. → Demo/train/eval segment lengths matched.
9. **DAgger can collapse an untouched skill** (bc2 lower/hold; dagger1
   lower 0/6). → Transition-DAgger rounds label EVERY segment
   (including lower) — this directly attacks the dagger1 lower
   collapse rather than inheriting it. Gate requires lower to REBUILD.
10. **DR confounds** (footlow2-level1): stacking new DR axes blind
    reopens closed exploits. → dual1's DR axes at 0.5, NO new axes.
11. **Cheat catalogue** (parked-leg paddle, two-foot park, sub-mm
    hover park, belly shuffle, crouch-splay): the eval gates that
    catch them (gait_valid, valid_plant, end_posture, park duty,
    prog_ratio) must apply PER SEGMENT in the sequence eval, not
    pooled — pooled numbers lie (start-kind lesson, 08-08).
12. **Erosion is dose-monotone** (noslip line): PPO erodes taught
    behavior its income disagrees with. → Stance anchors stay ON
    during arm 2's RL; sequence income must not price transitions so
    cheaply that parking through a segment wins.

## CODE items (land in this order; each default-off/bit-exact + tests)

1. **`goal.mode_seq` (walk_task/sim_env):** episode = K segments
   (K 2–5) drawn from the grammar
   `rise → {hold|walk} → {walk|lower} → (rise …)` with segment lengths
   `goal.mode_seq_segment_s` (default 6–8 s, jittered), episode
   `--episode-seconds` 25–30. At each switch: regenerate the goal traj
   for the new mode FROM THE CURRENT STATE (re-anchor `_z0`, href,
   ref arrays, BC-anchor eligibility — reuse the eval_handoff
   re-anchor mechanics), flip nothing else; blend window
   `goal.mode_seq_blend_s` (default 0.5–1.0 s) on the new segment's
   refs; per-mode reward kernels follow the ACTIVE segment; falls /
   over-current terminate as today. Start kinds: full existing
   diversity (belly flat/bridge, crouch, plant, mid-gait RSI) — a
   sequence may begin at any of them, first segment chosen compatibly
   (belly starts begin with rise; plant starts may begin with any).
   `goal.mode_seq=0` default = bit-exact current behavior.
2. **`distill_gru.py --transitions`:** demo episodes are teacher-CHAINED
   sequences: the composition-proven specialists drive segment by
   segment with the eval_handoff re-anchor at each switch; the student
   sees ONE continuous obs/act stream with the mode one-hot flipping;
   labels come from the active segment's teacher. DAgger rounds: the
   STUDENT drives whole sequences, active-segment teacher labels every
   visited state. Teacher choice must be VERIFIED in-context before
   collection (distill_gru already prints teacher returns; the rule
   stands — a teacher that scores badly in the sequence context cannot
   be distilled): default stance teacher `ppo_goal_cw_stance_dr10`
   (68-obs prefix, distills cleanly today); if the implementing cycle
   wants the composition-proven `ppo_goal_cw_stand_holdbc1_hard1` it
   must first confirm obs-layout compatibility (hist16 stack ≠ prefix)
   — do not assume.
   **LANDED (08-13, c-idlekick after the dual2 dig-in): implemented ON
   TOP OF item 1 rather than external re-anchor orchestration — the
   demo env runs `goal.mode_seq=1`, so the env itself chains grammar
   segments and re-anchors refs at every switch (identical mechanics
   to what arm 2 will train on — lesson 8, contexts matched), while
   `distill_gru --transitions N` routes per-tick teacher labels by the
   active segment (`_seq_teacher`) and records ONE continuous
   obs/act/mode stream per sequence. Teacher verification is a hard
   gate: the first `--seq-verify` (12) sequences run deterministic and
   collection ABORTS (SystemExit) past `--seq-verify-max-falls` (4).
   DAgger rounds collect whole student-driven sequences with
   active-segment teacher labels on EVERY segment incl. lower (lesson
   9). Default 0 = off, bit-exact. Tests:
   `rl_move/tests/test_distill_transitions.py` (routing/one-hot
   agreement, stream continuity, verify abort, dual guard, mode_seq
   guard); semantics bank + mode_seq/gru/bc_anchor/sim_env/vec suites
   re-run green; end-to-end tiny smoke (3 seq + 4 single-mode eps +
   1 sequence DAgger round, real teachers: 0 falls) saved a loadable
   zip. Arm 1 recipe is in the module docstring.**
3. **Sequence eval instrument (`eval_modeseq.py` or an
   eval_checkpoint mode):** drives a FIXED command schedule
   (rise→walk→sit→rise→walk, plus a det+sto pass), reports PER
   SEGMENT: falls, the segment's own mode criteria (walk gait_valid +
   vel_err + prog_ratio; rise/lower height-err + end_posture +
   valid_plant; hold park-duty), switch-window max tilt/current, and
   start-kind split. This is the gate instrument for BOTH arms —
   build and baseline it on the two-specialist composition (known
   zero-fall) before gating anything.
   **LANDED + baselined (08-13, c-triage): `rl_move/sim/eval_modeseq.py`
   (external orchestration via `reanchor_to`, generalizing
   eval_handoff{,_reverse}.py; zero env/reward touch). Baseline (the
   directive's own reference) is `footlow2_hard1` + `walk_longdist_r2`
   at the 5-segment grammar above, NOT `holdbc1_hard1` (which
   reproduces its known sit-after-walk defect: 7/12 zero-fall).
   footlow2_hard1: det 11/12 (rise 23/24, lower 12/12, walk 23/23),
   stochastic 9/12 — every stochastic sequence-ending fall landed on
   the SECOND (post-lower) rise (8/12 vs the cold first rise's
   10/12). This is the zero-fall noise band + the numbered
   start-relative-`_z0` risk items 1/2 must beat.
   Artifacts: `logs/ckpt_eval/modeseq_baseline_{det,footlow2_det,
   footlow2_sto}.json`.**
   **GATE-READY UPDATE (08-13 ~23:xx, pre-Arm-1-triage): the two
   gaps between the v1 instrument and the Arm 1 gate are CLOSED —
   (a) `--single <ckpt>`: ONE checkpoint drives every segment (the
   Arm 1/2 artifact path; v1 could only drive the two specialists).
   GRU hidden state persists ACROSS segment switches within a
   sequence (reset at true episode start only — the `--transitions`
   continuous-stream contract; per-switch resets would evaluate a
   memory lobotomy), dual-core obs one-hot auto-detected from the
   stored obs width (eval_checkpoint parity). (b) per-segment
   criteria per lesson 11: walk segment success now ALSO requires
   gait_valid (identical duty/swings/sacrificed formula to
   eval_checkpoint); prog_ratio + slip_per_m reported per segment;
   every segment reports switch_tilt_deg/switch_peak_a (max
   |roll|/|pitch| + peak servo current, first 1.5 s post-reanchor —
   the switch-window evidence field, no bar in v1). Smoked on
   train-1: specialist path reproduces the baseline (walk
   gait_valid 4/4, prog 1.05–1.11, slip 1.7–2.0 in-band; the
   post-lower rise's switch window reads 5.8–9.1° vs 0.3–2.6°
   elsewhere — the second-rise risk now has a per-switch
   instrument); `--single` on the dagger1 dual-GRU zip reproduces
   its known profile (walk honest gv 2/2, lower collapsed 0/2,
   post-lower rise tilt_roll falls). Baseline artifacts refreshed
   with the new fields:
   `logs/ckpt_eval/modeseq_baseline_footlow2_{det,sto}_v2.json`
   (train-1). Hold-segment park-duty stays unwired — no hold token
   in either arm's gate grammar; wire it if a grammar adds hold.**

## Arm 1 — `cw-arch-trans-dagger1` (transition DAgger distillation)

One model that does rise→walk→sit→rise→walk by imitation + DAgger.
Runs on idle pod CPUs (it is a CPU job; the fleet idles at 12/12) or
locally by the operator — implementer's call, artifact is what
matters.

- `distill_gru --dual --transitions`: ~300 sequence episodes (25–30 s)
  + the 08-13 stance-heavy single-mode mix as a retention floor
  (~200 eps, walk=0.30/rise=0.40/lower=0.15/hold=0.15 — keep
  single-mode competence while adding transitions), 30 epochs, then
  2 DAgger rounds × ~100 sequence episodes. log_std −1.5 on save (house
  rule). Artifact: `ppo_goal_cw_gru_dual_bc_transdagger1.zip`.
- **Gate (pre-registered):** on the sequence eval, det, DR0: zero
  falls in ≥11/12 sequence episodes AND every segment type passes its
  own criteria in ≥8/12 episodes AND single-mode retention vs dagger1's
  numbers: hold det ≥5/6, walk gait_valid ≥5/6, rise n=12/seed=1
  recheck ≥3/12 with ≥1 non-crouch win, **lower REBUILDS to ≥4/6**
  (the dagger1 collapse must not survive a distill whose demos contain
  real lower segments).
- **FAIL branches:** falls concentrated AT switches → the blend window
  is the lever (widen/reshape), not more demos; a single segment type
  fails while its single-mode twin passes → the re-anchor at that
  switch is buggy (instrument, fix, rerun — implementation, not
  science); lower still collapsed → drop the DAgger rounds on lower
  segments only (bc2 precedent) and re-distill.

**ARM 1 RESULT (08-14, first artifact `...transdagger1.zip`): FAIL on
the pre-registered gate — root cause is the DEFAULT STANCE TEACHER,
proven by a matched-teacher control, and the fix is running.**
Sequence eval det DR0: **0/12 zero-fall** (bar ≥11/12; sto 0/12 too).
Per segment: walk 12/12 gait_valid (prog med 0.79) — the rise→walk
switch transfers; rise 0/24 (first rise never falls but ends 15–25mm
short with feet unplanted); lower 0/12 falls-free but parks a WHOLE
TRIPOD (legs 0/2/4) 80–260mm in the air; the post-lower rise then
tips `tilt_roll` **12/12** at 12–14° switch-window tilt. Retention
(single-mode DR0): hold det 6/6 PASS; walk gait_valid 6/6 PASS (prog
1.03, slip/m 1.46; success letter 2/6 on the dagger1-init's same
vel_err 0.033-vs-0.030 polish miss); rise n=12/seed=1 det 3/12 with
3 non-crouch wins PASS (= dagger1 init); **lower det+sto 0/6 — NOT
rebuilt (bar ≥4/6), and with the TEACHER's signature: worst_clear
261mm (tripod parked in the air), not dagger1's belly-drag collapse**
— the broken in-context teacher lower out-voted the 30 single-mode
lower demos. Artifacts:
`logs/ckpt_eval/cw_arch_trans_dagger1_{gate,rise12}` (train-0).
**Matched control (same instrument, det, seed 0):** the default
stance teacher `ppo_goal_cw_stance_dr10` scores **0/12 zero-fall
itself** with the IDENTICAL fingerprint segment for segment (first
rise herr −14..−27mm; lower end_clear ~[90,85,250]mm on legs 0/2/4;
post-lower rise tilt_roll 12/12 at 12–15°) —
`logs/ckpt_eval/modeseq_stancedr10_det.json` vs
`transdagger1_modeseq_{det,sto}.json`. So `--transitions` distilled
with HIGH fidelity — the student is a faithful copy of an in-context
INCAPABLE teacher. The demo log agrees: the teacher fell 30/300
collection sequences (rise 24, lower 5, walk 1) — the directive's own
"a teacher that scores badly in the sequence context cannot be
distilled" condition was violated in the data, but the mechanical
`--seq-verify` (falls-only, first 12 det) PASSED because a tripod-up
lower does not fall. **Lesson for the ledger: the teacher-verify gate
must check segment QUALITY, not just falls — until then, pre-verify
any teacher on THIS instrument (footlow2_hard1 det 11/12 is the
passing reference; stance_dr10 det 0/12 is disqualified).**
**Fix attempt `cw-arch-trans-dagger2` (one variable:
`--stance-teacher` → `footlow2_hard1`, 68-obs compat proven) was
launched 08-14 and KILLED mid-collection by the same cycle — and the
kill is the cycle's biggest finding: `goal.mode_seq` (CODE item 1)
itself is now the prime suspect.** In the mode_seq TRAINING env the
footlow2_hard1+walk_longdist_r2 pair fell **99/225 demo sequences
(44%; lower 73, walk 22, rise 3)** with teacher return med ~298/min
−485 — while the SAME pair scores **11/12 zero-fall on the eval
instrument** (whose reanchor mechanics are the composition-proven
eval_handoff ones) and footlow2's lower never falls anywhere else
(12/12 single-mode AND in-sequence on the instrument). stance_dr10
shows the INVERTED pattern (30/300 in-env vs 0/12 on the
instrument). Two switch implementations, two contradictory verdicts
on the same checkpoints → the in-env per-switch re-anchor
(especially walk→lower) does not reproduce the proven handoff
context. The falls-only `--seq-verify` cap (12 det eps) passed at
4/12 and failed to protect the dataset (poisoned per this
directive's own teacher rule; no artifact was written).
~~NEXT (agent-doable, blocking both arms): instrument the mode_seq
walk→lower switch — dump the refs/_z0/q_nom/blend state the env
generates at the switch and diff against `reanchor_to()`'s on the
same physical state; fix item 1; re-verify footlow2_hard1 composes
in-env at ≈ its instrument rate; ONLY THEN re-run the transdagger2
recipe.~~ **DONE (08-14 ~02:xx UTC, operator session, local Mac) —
root cause found, fixed, re-verified; the CPU path is unblocked.**
The instrumented diff found it immediately: the v1 switch carried
the EPISODE-reset `q_nom` ("flip nothing else") and re-based `_z0`
on the instantaneous chassis height; `reanchor_to()` instead derives
`q_nom`/`_z0`/pad refs from a fresh settled reset of the TARGET
mode. Obs joints are `(q − q_nom)` and the settled belly vs plant
`q_nom` differ by **78.9° at the knees** — a rise-start sequence put
every later plant-family segment that far off the teachers' obs
distribution (hence lower worst at 73/225: never a first segment;
hence the stance_dr10 inversion: wrong-frame obs happened to mask
its post-lower weakness). Fix in `sim_env.py`: a reset()-time
settle probe (`_seq_capture_frames`) mints the canonical
plant+belly frames on the episode's own DR'd model, and
`_seq_maybe_switch` installs the target family's frame at every
switch; hold/lower BC base = that canonical `q_nom`; physics,
servo profile, slew memory and tilt frame still carry over.
Frame parity is locked as a regression test
(`test_canonical_frames_match_fresh_reset_frames`). **Re-verify
(exact collection context, DR0.5, seed 0,
`verify_modeseq_teachers.py`): footlow2_hard1+walk_longdist_r2
12/225 fell (5.3%; lower 5) vs 99/225 (44%, lower 73) on the pods
pre-fix — and a same-machine/seed pre-fix A/B replicates the pod
number at 92/225 (40.9%, lower 62), so the fix alone moves
41%→5.3%. Teacher return med 658 vs 287 — ≈ the instrument band.
Condition met.** Evidence:
`rl_move/sim/logs/verify_modeseq_footlow2_{prefix_ab,postfix}.json`
+ `verify_modeseq_stancedr10_postfix.json` (local). stance_dr10 control 9/300 (its tripod-up-lower quality
defect stands; teacher-quality gate lesson unchanged). Residual
scope: the MJX batched-reset path never runs env.reset(), so
`goal.mode_seq=1` there raises loudly until a batched frame mint
lands (named follow-up CODE item) — **arm 2 on MJX waits on that;
the transdagger2 recipe (CPU) can re-run NOW.** **BOTH DONE (08-14
~03:xx UTC, orchestrator): transdagger2 re-run RUNNING on train-0
(on-pod teacher verify 2/12 det falls, return med 656 — the fix
reproduces in the collection context on the pod), and the MJX mint
LANDED (`MjxVecEnv._mint_seq_frames`, commit 8374125: batched
choreography mints the canonical plant/belly frames per episode,
default-off, pod-verified frame parity vs fresh C reset + batched
switch crossing as regression tests). Arm 2's only remaining wait
is the transdagger2 artifact triage.**

**ARM 1 RE-RUN RESULT (08-14 ~12:4x UTC, artifact
`ppo_goal_cw_gru_dual_bc_transdagger2.zip`, md5 02748e27, train-0;
evals `logs/ckpt_eval/transdagger2_modeseq_{det,sto}.json`,
`cw_arch_trans_dagger2_{gate,rise12,lower}`): FAIL by the letter of
the pre-registered gate — but ONLY on the two rise clauses; every
fall-related and lower clause now PASSES, several above the
two-specialist baseline.** Scorecard: (1) sequence det DR0 zero-fall
**12/12 PASS** (bar ≥11/12; baseline itself 11/12; sto 10/12 vs
baseline 9/12 — the 2 sto falls are both FIRST-rise-from-flat
`over_current`, a new flavor, switch windows clean at ≤7.7° vs
dagger1's 14.5°). (2) per-segment: walk 24/24 gait_valid prog_med
1.05 PASS; lower 12/12 PASS; **rise: cold FIRST rise 5/12 det**
(misses end 4–17mm short + footprint, ZERO falls; the post-lower
rise is 12/12 — dagger1's killer switch, now the strongest segment;
the baseline's own risk INVERTED) → clause MISS. (3) retention: hold
6/6 PASS; walk gait_valid 6/6 det+sto (letter 1/6 on the same
vel_err polish miss as the init); **lower REBUILT 6/6 det + 6/6 sto,
worst_clear 0mm det (bar ≥4/6; dagger1: 0/6 with a 261mm airborne
tripod)**; **rise12 (n=12/seed=1) 3/12 meets ≥3/12 but ALL THREE
wins are crouch — 0 non-crouch (init had 3) → clause FAIL.** Video
(det seq strip ep0): honest flat→rise→tall six-leg walk→lower→
rise→walk, no parked leg, no belly shuffle, no fall. Verdict:
teacher-quality fix + canonical-frame fix SOLVED the sequence-fall
problem end to end; what the DAgger rounds traded away is the
non-crouch single-mode rise (sequence data is crouch/plant-heavy
relative to bridge/flat cold starts — a data-mix question, not a
mechanics one; none of the three pre-registered FAIL branches
matches, so no discretionary transdagger3 was launched). Per the
warm-start order below (no discretion at launch time): gate (1)
FAIL → (2) gru-dual2 FAIL → **(3) `ppo_goal_cw_gru_dual_bc_
dagger1.zip` — `cw-arch-modeseq1` QUEUED 08-14 with exactly that
init.** Candidate follow-up for the operator/next triage if Arm 2's
rise clause also misses: rise-only-DAgger variant distill (the
already-routed option (b)) or a bridge/flat-heavy `--transitions`
demo mix on the transdagger2 recipe.

## Arm 2 — `cw-arch-modeseq1` (consolidated RL, the "one good set" arm)

10M, dual-core GRU, `goal.mode_seq=1`. THE one-variable framing: the
recipe is dual1's proven stack — same DR axes at 0.5, same anti-cheat
reward config, stance anchors ON / walk-tick anchor OFF — with mode
sequencing as the ONLY new thing. No new reward terms in v1; no new
DR axes; no wider command set.

- **Warm-start order (pre-registered, no discretion at launch time):**
  (1) `cw-arch-trans-dagger1` artifact if its gate PASSED;
  (2) else `cw-arch-gru-dual2` checkpoint if its gate PASSED;
  (3) else `ppo_goal_cw_gru_dual_bc_dagger1.zip`.
- Starts: full diversity incl. mid-sequence spawns OFF in v1
  (sequence-RSI is a pre-registered follow-up lever, not v1).
- Episode 25–30 s; goal-mix becomes the sequence grammar; retain ~25%
  single-mode episodes (multitask lesson: keep the prior's own
  distribution in the diet).
- **Gate (pre-registered):** sequence eval det+sto DR0 + own-DR0.5:
  zero falls in ≥11/12 det sequences AND per-segment criteria ≥9/12
  AND single-mode retention at dual1 levels (walk gait_valid ≥5/6
  prog ≥0.80, hold ≥4/6, lower ≥4/6, rise n=12 method ≥ its own
  init's number) AND switch-window max tilt reported (baseline it —
  no bar in v1, evidence for the follow-up knob).
- **FAIL branches:** walk freezes only inside sequences → cross-segment
  interference through the shared feature extractor (the one part both
  cores share) — lever is extractor split/detach, CODE not config;
  falls at switches with clean segments → the pre-registered
  switch-window tilt charge (ONE knob) or entry-slew analog on mode
  switches; rise-in-sequence fails while single-mode rise passes →
  sequence-RSI (spawn mid-sequence, existing RSI precedent from
  tall-rsi1 applies: expect recovery-robustness, verify it doesn't
  just learn to dive to a safe mode).

**ARM 2 RESULT (08-14 ~17:xx UTC, `cw-arch-modeseq1-r1`, warm from
init (3) per the order above, canary auto-stop 4.56M/10M): FAIL —
sequence det DR0 zero-fall 2/12 (bar ≥11/12; sto 3/12), all falls
inside rise segments (flat cold-start 0/3 all fell; post-lower rise
2/7; switches clean ≤8.2°; walk 9/9 gv prog 1.014; lower 7/7).**
rise12 recheck det 5/12 = crouch 5/5 / bridge 0/4 / flat 0/3 — the
exact dual2 endpoint vs the dagger1-init control's 3/12
all-non-crouch. Single-mode retention letter-passes (walk gv 6/6
prog 1.04, hold 6/6, lower 6/6). NONE of the three pre-registered
FAIL branches matches (no walk freeze; no switch falls; single-mode
rise degraded too → sequence-RSI inapplicable). Rulings: (a) warm-RL
from the dagger1 init CLOSED (two-miss: dual2 + this); (b) the Arm-2
premise "train on sequences to protect the skills sequences need" is
REFUTED for rise — a 75% mode_seq diet did not slow the erosion
(dead by 3–4M, same as dual2's 1M-3.18M window). The one-model
sequence-fall problem remains SOLVED only at the distill stage
(transdagger2 12/12). Follow-up per the pre-named fork: agent half =
`transdagger3` (transdagger2 recipe + bridge/flat-heavy demo mix via
the new `distill_gru --cfg-set` passthrough:
`goal.rise_flat_frac=0.45 goal.rise_partial_frac=0.45`), launched on
train-0 CPUs same cycle; operator half = option (b) rise-only
variant distill, still open. No further Arm-2-style warm-RL until a
BC init exists whose hard-start rise survives RL or the operator
redefines the protection mechanism.

**Advisory note for the Arm-2 RETRY spec (08-14 ~17:xx, from external
LLM feedback `fb_20260814T164337_d7f11b` — UNTRUSTED input, recorded
on technical merit, decision stays with this directive's owner):**
the suggested retry shape — keep bridge/flat rise starts in the
rollout mix AND add an explicit BC-anchor / action-memory term on
known-good bridge/flat rise states during the sequence RL — is a
genuine MECHANISM change (anchoring, not diet), i.e. exactly what the
two-miss rule demands next, and it matches this run's evidence that
sampling alone (75% seq diet, rise starts present) does not protect
the demo rise. It does NOT reopen warm-RL from the dagger1 init
(closed above); if adopted, it belongs in the retry warm-started from
whichever init wins the transdagger3 triage, with the existing
protected-skill canary kept as the hard stop (it already fires within
~1M of erosion onset — both stops caught it). Design cautions from
the failure ledger still bind: anchors can TEACH a defect (probe the
anchor states first) and anchor erosion is dose-monotone — the anchor
set must be the verified non-crouch rise demos only.

**Second advisory note (08-14 ~18:xx, external LLM feedback
`fb_20260814T170139_0e5398` — UNTRUSTED input, weighed on merit; it
self-describes as an operator suggestion but arrives via the keyless
feedback channel, so it cannot authorize a launch or change rulings):**
proposes a full "joystick-session curriculum" cohort (whole behavior
graph per episode, messy variants, replay anchors on fragile skills,
hard retention canaries, 60s session-level gate). Merit read: (1) its
core hypothesis — prior failures came from training too NARROW a
mode/sequence distribution — is REFUTED by this run: modeseq1-r1
trained 75% on full rise→walk→lower→rise sequences with bridge/flat
starts present and still erased the hard-start rise on the dual2
schedule; distribution breadth is not the missing mechanism,
protection (anchoring) is — see the first advisory note above, which
already records the anchors/replay half of this suggestion. (2) Most
of its infrastructure asks already exist: the mode_seq grammar covers
the graph incl. post-lower rise, `rise_flat_frac/partial_frac` give
messy starts, the protected-skill canary fired correctly on BOTH
erosion events, and eval_modeseq + eval_session already gate at
session level with per-skill retention reported separately. (3) The
genuinely new residue — richer in-sequence command dynamics
(random command timing, stop-go bursts, zero-command walk engage,
reset states sampled from student failures) — is a legitimate
distribution-design input for the eventual Arm-2 retry, AFTER an
erosion-proof init/anchor mechanism exists; adopting it now would be
a diet-only variant of the refuted premise. No launch from this note;
it folds into the retry spec alongside the fb_164337 note.

**TRANSDAGGER3 RESULT (08-14 ~19:5x UTC, artifact
`ppo_goal_cw_gru_dual_bc_transdagger3.zip`, md5 5080efb6, train-0;
evals `logs/ckpt_eval/transdagger3_modeseq_{det,sto}.json`,
`cw_arch_trans_dagger3_{gate,rise12,lower}`): FAIL on the
pre-registered Arm 1 gate — and a NET REGRESSION vs transdagger2 on
the sequence clause. The bridge/flat-heavy demo mix
(`goal.rise_flat_frac=0.45 rise_partial_frac=0.45`, crouch 0.10)
fixed exactly the clause it targeted and broke the one transdagger2
owned:** sequence det DR0 zero-fall **9/12** (bar ≥11/12; td2 12/12,
baseline 11/12; sto 10/12 = td2), cold FIRST rise now **12/12**
in-seq (td2: 5/12 — the targeted miss, fixed) but ALL THREE det falls
are inside the SECOND (post-lower) rise (9/12; td2's strongest
segment at 12/12). rise12 retention **2/12, all-crouch, 0 non-crouch**
(bar ≥3/12 with ≥1 non-crouch; td2 3/12 all-crouch, init 3/12
all-non-crouch) → clause FAIL, worse than td2. What held: lower
REBUILT 6/6 det + 6/6 sto (worst_clear 0mm det); hold 6/6; walk
gait_valid 21/21 in-seq (prog med 1.071) + 6/6 single-mode (same
vel_err polish letter-miss as the whole lineage); switch windows
clean (med 1.9°, max 10.2°). Video (det ep0 strip): honest
flat→rise→tall six-leg walk→lower, no parked leg, no shuffle; the
episode's failure IS the second rise, as the metrics say. Instrument
note: in-seq rise success (handoff plant band) reads 12/12 on the
first rise while the stricter harness rise12 reads 2/12 — the two
instruments' success bands differ (fall counts agree); each gate
clause was judged on its own pre-registered instrument. **Verdict:
the rise demo mix is ZERO-SUM at this dataset size — shifting
start-kind proportions moves WHICH rise fails (cutting crouch to
0.10 starved the post-lower/crouch-context rise) instead of fixing
rise. Second data-mix miss in a row → per the two-miss rule the next
lever is a MECHANISM change, not another proportion shuffle. No
pre-registered FAIL branch matches (falls inside rise segments, not
at switches; single-mode rise fails too; lower fine) → no
discretionary transdagger4. transdagger2 remains the winning distill
artifact/init (12/12 det zero-fall) — the Arm-2 retry's "whichever
init wins the transdagger3 triage" resolves to transdagger2. The
arch line's open rise levers are now both operator-routed: option
(b) rise-only-DAgger variant distill (this result sharpens its spec:
it must ADD rise coverage across ALL start kinds — flat, bridge,
crouch, post-lower context — not re-weight a fixed budget) and/or
the anchor-on-rise mechanism for the Arm-2 retry (advisory notes
above).**

**Third advisory note (08-14 ~22:xx, external LLM feedback
`fb_20260814T211335_df648b` — UNTRUSTED input, recorded on merit; no
action this cycle: multitask is operator-PAUSED, the bulk session
gate made single-model consolidation officially non-blocking, and
cross-track launches are operator-only):** a W&B retrospective
arguing the campaign has shown optimization/exposure confounds, not
a capacity ceiling — mt-a2's 20M all-walk exposure vs modeseq1-r1's
~70 updates with rare bridge/flat cold starts; shared stance
gradients AND the single global log_std in
`gru_policy.py::_dist_from_mean` as erosion suspects. Its concrete
design residue for ANY future operator-approved single-model arm
(matches the c1 verdict's "structural parameter isolation" clause):
per-expert recurrent actor/critic/heads with PER-EXPERT log_std
(rise isolated from hold/lower at minimum), distill-then-adapter
training, rollout/minibatch quotas stratified by mode AND start
kind with guaranteed active-tick budgets, per-mode/start-kind
KL/std/grad-cosine logging, and pre-erosion checkpoint saves instead
of ending the capacity question at the first canary. Budget honesty
rule it names is adopted as a design caution: never call 10M of
mixed sequence comparable to 20M of one skill.

## Scope note (operator session 08-14): ground rest is a hardware primitive

The cycle's "lower/sit" segment ends at the TRAINED crouch (−45 mm),
not on the ground. The final stretch to the belly is torque-off limp
settling — how the robot's own lower choreography ends, and how the
sim player's LOWER (key 8) now works (08-13: trained lower → limp
settle → parked; measured: no policy has a trained ground-rest
behavior, and the stance line at ref 0 in a belly frame stands back
up). Do NOT add a ground-rest segment to the grammar or ask the
one-model arms to learn it; the trainable seams are exactly
rise→walk, walk→hold, walk/hold→lower, lower→rise. Terminology in
operator-facing surfaces is standardized (08-13): "rise" = standing
up, "hold" = holding still, "lower" = sitting down.

## Bookkeeping

Same rules as every arm: ledger entries with hypothesis + gate at
queue time, harness/sequence eval before any promotion, wandbnotes,
track STATUS updates. The sequence eval baseline on the two-specialist
composition is itself a recorded artifact (it defines the zero-fall
noise band the one-model arms must live inside).
