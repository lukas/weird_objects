# arch — Advanced architectures

W&B: tag `track:arch`. Excess-capacity research.

**Goal:** get a more advanced model (GRU/recurrent/temporal) to walk,
stand up, and sit down. What architecture learns the full skill set,
at what budget, with which failure modes.

## Now

- **08-13 ~06:xx: the parity audit RAN and EXONERATES the warp
  physics — the no-slip line's story is corrected and the line
  CONCLUDES at r4.** `probe_contact_parity.py` (new, snapshot
  ac5500a) drove the identical TripodGait command stream through the
  identical servo-profile pipeline from one shared settled start in
  {C@50, C@1/4, warp@1/4, 2/4, 4/8, 8/8}: loaded-foot slip warp@1/4
  is within ~6% of C@50 at 0.055 m/s and ~3% at the line's own
  0.012 m/s band, flat across the iteration sweep, and pure stance
  load creeps ZERO in warp (0.0000 m vs C's 0.0007 m); C@1/4 itself
  goes NaN-unstable, so warp's 1/4 is a different, stable solver, not
  truncated C. **Re-attribution of dr0's "0.085 MJX vs 0.31 C":** the
  0.085 was the stochastic on-policy TRAINING metric, the 0.31 a
  deterministic probe — the prior income audit's own C stochastic
  replays measured ratio 1.42–1.45 m/m ≈ MJX's ~1.44 — and the steep
  loadslip-factor clip (ls_ok 0.75/ls_max 1.5) amplifies the ~13% raw
  ratio difference (1.44 vs 1.27) into a 3.6× factor difference.
  What SURVIVES: PPO's anchoring erosion is real and dose-monotone in
  DET behavior too (2M→0.11, 1M→0.31–0.45, 500k→0.54 factor) under
  bank-verified pricing in a clean world — an RL-incentive fact, not
  a sim defect, and per the two-miss rule the fix is a mechanism
  change, not more optimizer/dose arms. **r4 stands as the line's
  artifact (GATE PASS, preserves the taught no-slip gait; adds ~zero
  under DR); no training arm is queued** — genuine RL gains on this
  line need a new mechanism (the operator's DAgger redistillation
  thread, or an erosion-proof anchoring design), which is spec work.
  Audit data: train-0 `logs/probe_contact_parity/parity*.json`;
  earlier income audit `logs/probe_noslip_income/`. The hw
  cross-track escalation is RETRACTED (corrected in hw/STATUS.md).
- **From-scratch GRU walking CLOSED (08-11, gru-r4c):** both
  pre-registered levers (BPTT window 64→256 steps, hidden 128→256)
  tried and the identical leg-sacrifice/paddle fingerprint survived
  (legs [0,2,3] parked, det gait_valid 0/6) — worse, rise also
  regressed to 0/6 (r3 had it at champion grade). Reward already
  works on the MLP lineage, so this is a capacity/architecture limit,
  not a pricing gap. No further from-scratch GRU variant.
- Frame-stack line passed: hist16 → hist16-dep1 (deploy contract).
- **BC-distill-then-RL-finetune tried (08-11, `cw-arch-gru-bc-ft1`):
  walk survives, stance is MIXED, and a dig-in corrected the root
  cause.** The GRU was BC-distilled from the walk+hold specialists
  (rise was NEVER in the distillation data — the BC parent itself is
  a hold+walk artifact only) then PPO-finetuned 10M steps on a
  walk-heavy mix (60/15/15/10). Result across all four gate+own-DR0.5
  passes: walk gait_valid 6/6 throughout, no paddle at any point in
  10M steps (economy softened, slip/m 2.1-2.6 vs BC parent 1.5);
  lower IMPROVED (4-6/6 vs parent 2/6 det); rise stayed 0/6 (unchanged
  from the BC parent — RL never invented a skill it had zero demos
  for, not forgetting); hold REGRESSED 0/6 det (parent was 6/6 — RL
  genuinely traded hold precision for locomotion polish, real
  forgetting this time). So two different mechanisms in one run:
  data-poverty (rise, never learnable this way) vs erosion (hold,
  actually lost). Keeping `ppo_goal_cw_gru_bc.zip` (md5 864c02fb) as
  the hold+walk reference artifact.
- **lr/KL lever CLOSED (08-11, `cw-arch-gru-bc-ft2`, 2M discovery)
  — FAILS exactly as pre-registered.** 3x lower LR (3e-4→1e-4) +
  tighter target-KL (0.02→0.01) on the identical BC-parent finetune:
  gate (DR0) det rise 0/6 (bridge/crouch/flat all 0, honest stalled
  climb, plant_margin 122mm short, balanced duty — not a cheat), det
  hold 0/6 (all 6 eps fail valid_plant on height/feet_down/footprint,
  worst foot 19-33mm proud, balanced duty — honest precision loss,
  not a flag-leg). Walk stayed clean (gait_valid 6/6 det+sto, slip/m
  1.3-1.7 in-band). Two attempts now (ft1 default hp, ft2 tight
  hp) at protecting stance via PPO hyperparameters alone — both lose
  it identically. **Per RESEARCH_RULES "two misses in the same
  behavioral class": stop tuning lr/KL, change the mechanism.**
  **Root blocker found, CODE not config:** the gate's own
  pre-registered fix ("an auxiliary BC-anchor loss during the
  finetune, or freezing early layers") is not available for this
  network — `train_ppo_mjx.py` explicitly raises `SystemExit` for
  `--gru` + `train.bc_anchor_coef>0` ("not implemented, wraps stock
  PPO"), and there is no layer-freeze flag either. The MLP-lineage
  BC-anchor (`cw-stand-bc1`/`holdbc1`, twice-proven on hw track) does
  not reach RecurrentPPO as written.

- **CODE landed (08-11 night): `bc_anchor.py` now supports
  `RecurrentPPO`/GRU** (hidden-state-correct aux step — anchors at
  the rollout's own hidden state, not h=0). Unblocks the CODE item
  below. Two arms ran on it 08-12:
- **`cw-arch-gru-anchor1` (10M hardening, warm from the ft1
  BC-distilled GRU, rise+hold+lower+walk all anchored) FAILS —
  but informatively.** Hold det 6/6 and lower det 6/6 (up from ft1's
  0/6 and 4/6) and rise det 2/6 with a non-flat start all clear their
  bars: **the anchor DOES protect/recover stance skills on a
  recurrent net**, exactly as it does on the MLP lineage. But walk
  freezes solid (gait_valid reads 6/6 with zero legs literally
  parked, yet prog_ratio 0.01, speed 0.001 m/s, video pixel-static
  for the full 15s clip) — the SAME twice-closed BC-anchor-on-walk-
  ticks failure (`cw-walk-gaitbc1`, `cw-omni-transbc1`), now
  reproduced on a GRU. Gate required both; overall FAIL.
- **`cw-arch-gru-scratch-anchor1` (2M discovery, from-scratch GRU +
  the same anchor stack) FAILS per its own pre-registration:** the
  anchor loss converges cleanly (0.010, under the 0.02 bar — a live,
  working teaching signal) but the walk mode still finds the
  parked-leg paddle (det gait_valid 0/6, leg idx1 flagged sacrificed)
  — "paddle beats a live anchor" was the pre-registered CLOSE
  condition. **From-scratch-GRU-with-anchor is CLOSED**; distill-
  then-finetune (ft1, warm from a BC-distilled net) remains the only
  path that has kept a GRU walking cheat-free.
- **`cw-arch-gru-anchor2` (10M, one variable off anchor1: walk-tick
  anchor OFF, rise/hold/lower still anchored) FAILS — the exact
  false-branch prediction confirmed.** Turning off the walk-anchor
  term did NOT unfreeze walk: det gait_valid still reads 6/6 but
  prog_ratio 0.01, speed 0.002 m/s, video pixel-static across all 10
  sampled frames both det+sto — identical to anchor1's fingerprint.
  Hold/lower held anchor1's levels (det 6/6 each); rise slipped to
  1/6 (below the >=2/6 bar, anchor1 had 2/6). **Proves the
  interference is the SHARED recurrent trunk (feature extractor + GRU
  cell every mode's forward pass runs through), not the walk-anchor
  loss term itself** — two straight misses on the binary on/off
  anchor lever, so per RESEARCH_RULES the mechanism changes, not the
  toggle. CODE landed same cycle: `train.bc_anchor_detach_trunk`
  (default off/bit-exact, tests in `test_gru_policy.py`) stops the
  anchor's gradient at the GRU/feature-extractor output so stance-
  tick anchoring only trains the actor head, never the shared trunk.
  Follow-up `cw-arch-gru-anchor3` (2M discovery, one variable off
  anchor2: `bc_anchor_detach_trunk=1`) RAN — see below.
- **`cw-arch-gru-anchor3` FAILS — the pre-registered false branch,
  decisively (08-12).** Detaching the BC-anchor gradient from the
  shared GRU trunk did NOT unfreeze walk: det gait_valid 4/6 with
  leg idx0 flagged sacrificed, prog_ratio 0.03 (need >=0.80),
  video pixel-static (no floor translation) at both DR0 and own-DR
  0.5, det+sto. Hold/lower both held clean (det 6/6 each; DR0.5 hold
  6/6, lower 4-5/6) — the trunk-detach mechanism DOES protect stance
  skills exactly as designed, which proves the walk freeze is NOT
  the anchor's gradient leaking through the trunk after all. **Three
  independent levers now refuted (anchor1 on, anchor2 walk-anchor
  off, anchor3 trunk-detached) — the anchor-for-recurrent-nets line
  (teaching a shared-trunk GRU to walk while anchoring stance ticks)
  is CLOSED FOR GOOD.** No further BC-anchor coefficient/toggle
  variant on this mechanism.

- **CODE landed + tested (08-12): mode-gated dual-core GRU
  (`DualGruActorCriticPolicy`, commit 2137c00) — separate
  locomotion/stance cores, routed per tick by `obs.mode_onehot`, so
  each core gets gradient exclusively from its own skill family.**
  `cw-arch-gru-dual-scratch1` (2M, from-scratch + full anchor stack)
  FAILS its own gate on one narrow clause (rise sto 2/6 vs the
  required >=3/6, n=6 — det rise unchanged at parent's 1/6) but
  DECISIVELY confirms the mechanism: det walk gait_valid 6/6 with
  **ZERO sacrificed legs** (parent `scratch-anchor1`: 0/6, one leg
  parked in literally every episode) — core isolation removes the
  shared-trunk interference that anchor1/2/3 could not. Hold/lower
  both hold parent's 6/6; anchor loss converges clean (~0.01).
  Residual to watch: under own-DR 0.5 the leg-sacrifice partially
  reappears (gait_valid 3/6 vs parent's 5/6, legs 0/2 occasionally
  parked) — not gate-breaking (gate is DR0 by design) but a sign the
  isolation is not yet perfectly robust under randomization.
  `cw-arch-gru-dual1` (10M, warm from the dual BC-distill, walk-tick
  anchor OFF / stance anchors ON) is training NOW on the same
  architecture — this is the actual "can a dual-core GRU walk while
  keeping stance" test (dual-scratch1 was diagnostic-only, not
  expected to displace at 2M). Its result decides this line, not new
  CODE.

- **`cw-arch-gru-dual1` FINISHED (08-12): the walk-freeze question is
  answered YES, decisively, at hardening scale — FAIL on the letter
  of its pre-registered n=6/seed=0 gate draw, PASS on the science,
  and rise is much closer to solved than that draw first showed.**
  Det walk: gait_valid 6/6, **zero sacrificed legs**, prog_ratio med
  0.95 (parent anchor3: 0.03, pixel-static freeze) — real six-leg
  translation confirmed on video, roll settled 6/6, roll_tail
  1.0–2.3°. Hold det 6/6 (drag 55mm, roll_tail 0.0° — BETTER than
  anchor3's 117mm/0.3°) and lower det 6/6 (drag 99mm vs anchor3's
  310mm) — both at champion level AND with better drag/roll than the
  shared-trunk parent, not just matching it. Rise on the gate's own
  n=6/seed=0 draw: 1/6 (bridge 0/3, crouch 1/1, flat 0/2), one
  episode short of the >=2/6 bar. **A same-cycle recheck (n=12,
  seed=1, identical DR0 gate cfg) found 7/12 (58%) with real bridge
  (1/4) and flat (1/3) wins, not just crouch (5/5)** — the small
  first draw was unlucky sampling, not a true sub-2/6 rate. Anchor
  loss converged clean (0.0034, same order as anchor1-3): not a
  routing/gradient bug. **Verdict: mode-gated dual-core routing is
  the mechanism that ends the shared-trunk interference — confirmed
  at 2M (dual-scratch1) and now at 10M (dual1) — and this checkpoint
  is closer to the full-skill bar than its own gate draw showed, but
  is not yet formally re-passed at a pre-registered larger n.**

- **`cw-arch-gru-dual-hfloor1` FINISHED (08-12): the plateau-fix
  lever does NOT transfer from the MLP lineage — it makes rise
  WORSE, with a new failure mode.** Same one-variable floor
  (`train.bc_anchor_min_h_ahead_mm=15`) that took the MLP stance
  lineage's rise from 3/6 to 12/12, warm from dual1. Per the "whoever
  triages it" note below, ran the identical n=12/seed=1 recheck used
  to correct dual1's own noisy n=6 draw: rise is 5/12 det (crouch
  5/5, bridge 0/4, flat 0/3 — **zero** non-crouch wins) and 1/12 sto,
  both WORSE than dual1's matched-method draw (7/12 det, 4/12 sto,
  each with real non-crouch wins). New pathology: 3-4 of the
  non-crouch episodes now trip `over_current` (0 in dual1's identical
  draw) — video-confirmed as an honest stall (splayed, motionless
  low crouch) running out of current headroom over the full 15 s,
  not a thrash or a new cheat. Walk/hold/lower all held clean: det
  walk gait_valid 6/6, zero sacrificed legs, prog_ratio med 0.99,
  real translation on video (dual-core fix intact); hold det 6/6
  valid_plant, duty 0.99-1.0, drag 48mm (vs dual1 55mm); lower det
  6/6, duty ~1.0, drag ~90mm (vs dual1 ~99mm) — stance quality
  slightly better, not worse. One cost: own-DR0.5 lower/sto picked up
  2 new tilt_roll falls (4/6 vs dual1's clean 6/6). **Verdict: this
  is the pre-registered FALSE branch, decisively — the anchor-shape
  plateau fix is MLP-lineage-specific, not a general fix for
  state-aligned/lookahead anchors. No further coefficient/floor-mm
  variant on this arm.** The rise gap here reads as data-poverty in
  the BC-distill (never enough real rise demos), matching the
  ft1/ft2 mechanism, not a supervision-aim problem.

## Next

- **The DAgger redistillation LANDED (08-13 ~13:00, operator session,
  local Mac): rise is in the BC init for the first time.**
  `distill_gru --dual`, stance-heavy mix walk=0.30/rise=0.40/
  lower=0.15/hold=0.15, 400 BC eps + 2 DAgger rounds ×150 (dataset
  698 eps, BC actor RMS 0.1865). Artifact
  `ppo_goal_cw_gru_dual_bc_dagger1.zip` (md5 b5167c10). Harness-eval
  before any RL (gate-cfg replica, DR0, det+sto): matched n=12/seed=1
  rise recheck **det 3/12 with real non-crouch wins (bridge 2/4, flat
  1/3), sto 2/12** — the old dual BC parent had effectively none
  (dual1's 7/12 needed 10M RL+anchors on top). hold det 6/6 retained;
  walk gait honest (gait_valid 6/6, prog_ratio 1.03, slip/m 1.45) but
  misses the success letter on tracking (vel_err 0.034 vs 0.030 —
  polish, RL's job). **Cost: lower collapsed det+sto 0/6** (drag
  550mm) — the bc2 DAgger-collapse fingerprint, on lower only this
  time. **Operator DECIDED (08-13 ~20:0x UTC): option (a) first —
  `cw-arch-gru-dual2` QUEUED to backlog** (10M, exact dual1 recipe,
  ONE variable: warm from this zip; ckpt pushed to all 12 train
  pods md5-verified; gate pre-registers the n=12/seed=1 rise
  recheck >=8/12 with >=2 non-crouch wins + lower REBUILD >=4/6 +
  walk retention). Every FAIL branch routes to option (b), the
  rise-only-DAgger variant distill (operator/local lever — lower
  kept BC-only), NOT a coefficient variant. Eval artifacts
  `logs/ckpt_eval/gru_dual_bc_dagger1_{gate,rise12}` (local).
- **OPERATOR DIRECTIVE (08-13 ~21:00 UTC): two jobs — ONE model for
  operator-commanded rise→walk→sit→rise cycles. Spec of record:
  `TRANSITIONS_DIRECTIVE.md` (this dir). Agent-doable, NOT
  operator-gated.** In order: land the three CODE items
  (`goal.mode_seq` mid-episode mode switching with per-switch
  re-anchoring of `_z0`/refs — the #1 hidden-state trap;
  `distill_gru --transitions` teacher-chained sequence demos via the
  eval_handoff re-anchor mechanics; a per-segment sequence eval
  baselined on the known zero-fall two-specialist composition), then
  run **arm 1 `cw-arch-trans-dagger1`** (transition DAgger distill,
  CPU job, gate incl. lower REBUILD ≥4/6 and zero falls ≥11/12
  sequences) and **arm 2 `cw-arch-modeseq1`** (10M consolidated RL:
  dual1's exact proven stack with mode sequencing as the ONLY new
  variable; warm-start order pre-registered trans-dagger1 > dual2 >
  dagger1 BC; gates + FAIL branches pre-registered in the directive).
  The directive's failure ledger (12 measured lessons) is binding —
  do not re-litigate closed levers inside these arms.
- **CODE item 3 LANDED + BASELINED (08-13, c-triage): the sequence
  eval instrument (`eval_modeseq.py`, new file, pure external
  orchestration of already-trained checkpoints — touches zero env/
  reward code, no semantics-bank exposure).** Generalizes both proven
  pairwise handoffs (`eval_handoff.py` rise→walk, `eval_handoff_reverse.py`
  walk→lower) into one `reanchor_to(mode)` helper applied N times for
  a 5-segment grammar (rise→walk→lower→rise→walk — the directive's own
  fixed schedule). **Baseline finding: the reference composition must
  be `footlow2_hard1`, NOT the deployed `holdbc1_hard1`** — holdbc1_hard1
  scores only 7/12 zero-fall end to end (n=12, det, seed 0; its known
  sit-after-walk stall from CURRENT_TRUTHS reproduces cleanly here:
  lower success 1/8 despite 0 falls, rise falls 5/20 with the crouch-tip
  fingerprint), while footlow2_hard1 clears the directive's own
  ≥11/12 bar exactly: **11/12 det** (seed 0; rise 23/24, lower 12/12,
  walk 23/23) and **9/12 stochastic** (seed 1; below the bar — all 3
  sequence-ending falls landed on the SECOND rise specifically, 8/12
  vs the first rise's 10/12). This is a real, decisive instance of
  failure-ledger risk #4/#5 (the two stances differ; start-relative
  `_z0`): even the BEST available stance specialist shows the
  reanchored post-lower rise measurably weaker than a cold-start rise
  under action noise — a concrete number for CODE items 1/2's authors
  to beat, not just the qualitative risk. Artifacts:
  `logs/ckpt_eval/modeseq_baseline_footlow2_{det,sto}.json` +
  `modeseq_baseline_det.json` (the holdbc1_hard1 control). **Next:
  CODE items 1 (`goal.mode_seq`) and 2 (`distill_gru --transitions`)
  are still OPEN** — scoped this cycle but NOT attempted: the MJX/warp
  vectorized stack turns out to reuse the exact same per-env Python
  objects as the CPU path (`mjx_host.SNAP_ATTRS` pool-restores plain
  `sim_env`/`walk_task` attributes), so item 1 is tractable as a
  `walk_task`/`sim_env` change (NOT a new MJX-side mechanism) — but it
  requires carefully splitting `_reset_finalize`'s ~600-line "physical
  reset" and "goal-derived per-episode state" halves so a mid-episode
  switch can re-run only the latter (re-anchor `_z0`/`_h_target`/
  `_plant_feet_xy`/BC-anchor eligibility flags from the CURRENT
  physical state, add new per-episode attrs to `SNAP_ATTRS`, index
  `_current_goal()` by a segment-local offset default-0). That is
  real, testable, contained work for a dedicated cycle — not
  something to rush alongside a triage pass. Item 2 (`--transitions`)
  can actually proceed WITHOUT item 1 (demo collection is external
  orchestration, same `reanchor_to` trick this instrument just
  proved out) and is the better next CODE step.
- Distill-then-finetune (`ft1`, warm from a BC-distilled net) is
  SUPERSEDED as the walk-freeze workaround — dual-core is strictly
  better (walk retains AND hold/lower beat it on drag) — but keep
  `ppo_goal_cw_gru_bc.zip` as the hold+walk reference artifact per
  the "Now" entry above.
- Later: contact-from-proprioception aux head.

Detail: RL_PLAN.md "Architecture" · ledger cw-arch-* lineage.
