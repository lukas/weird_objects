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
   start-relative-`_z0` risk items 1/2 must beat. Per-segment
   mode-specific criteria (gait_valid/prog_ratio/park-duty/switch-
   window tilt) are NOT yet wired — current success is a coarser
   fall+posture+tracking check; tighten when items 1/2 need it.
   Artifacts: `logs/ckpt_eval/modeseq_baseline_{det,footlow2_det,
   footlow2_sto}.json`.**

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

## Bookkeeping

Same rules as every arm: ledger entries with hypothesis + gate at
queue time, harness/sequence eval before any promotion, wandbnotes,
track STATUS updates. The sequence eval baseline on the two-specialist
composition is itself a recorded artifact (it defines the zero-fall
noise band the one-model arms must live inside).
