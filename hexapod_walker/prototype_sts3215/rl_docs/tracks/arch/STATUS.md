# arch — Advanced architectures

W&B: tag `track:arch`. Excess-capacity research.

**Goal:** get a more advanced model (GRU/recurrent/temporal) to walk,
stand up, and sit down. What architecture learns the full skill set,
at what budget, with which failure modes.

## Now

- **08-15 ~15:0x UTC — `cw-arch-tf-r1-hard1` (40M hardening twin)
  TRIAGED PASS: the causal-transformer trunk GROWS a real walking
  gait at budget parity with the hist16-MLP champion (r7).** DR0
  gate det+sto gait_valid 6/6, zero sacrificed legs, zero falls
  (terms 0), prog_ratio med 1.14/1.08 (>=0.85 bar); own-cfg DR0.5
  same 6/6/6/6, 0 term, prog med 1.10/1.00. Roll behavior clean
  every episode (roll_class clean/recovered, peak 2.0-7.8°, tail
  0.4-1.9°) — NOT the universal takeoff-roll-transient fall pattern,
  and a clear jump from this same arm's 2M canary (fell via roll on
  every det episode). Video (10-frame strips, all 24 eps across both
  DR passes) shows six feet cycling through changing contact
  patterns over the 15s clip, no flag leg, no static/parked gait.
  Slip is elevated (med 1.60 det/1.53 sto) vs r7's freshest read but
  sits inside the range r7's OWN verdict already flagged as
  "elevated vs contract-line champions, not gated here" (1.3-1.6) —
  not a new regression, the same documented economy gap this whole
  architecture line carries. **ASSUMPTION (operator to review): no
  bulk_session_eval cohort before this verdict** — the checkpoint is
  confirmed obs-INCOMPATIBLE with eval_session/bulk_session_eval
  (1152 vs 72, same obs-contract mismatch every arch-line checkpoint
  since r7 has had), so the pre-registered 6-episode-per-mode
  harness gate (the same convention r7 itself was verdicted on) was
  treated as the applicable evidence bar for this obs family — same
  precedent as every other arch-line walk verdict already in
  SKILLS.md. **Answers the arm's question cleanly: a small attention
  trunk is not a dead end — given the same 40M budget the MLP
  champion needed, it gets to the same place.** Walk-only; rise/
  hold/lower untested on this trunk. No further tf-line fork is
  pre-registered — a full-skill (rise/hold/lower) extension on the
  transformer trunk is the natural next question but needs its own
  spec (BC-anchor/composition choice), not a plain respec; left for
  a future cycle rather than launched speculatively this cycle.
  Evidence: `rl_docs/runs/cw-arch-tf-r1-hard1.md`,
  `logs/ckpt_eval/cw_arch_tf_r1_hard1_{gate,owncfg,session}`.
- **08-15 ~13:4x UTC — `cw-arch-tf-r1b` TRIAGED PASS (2M discovery
  mechanism canary): the causal-transformer PPO stack boots/trains
  cleanly to 2M (no NaN/crash, ep_rew -1.5→93, EV 0.02→0.86, std
  stable 0.37→0.39), det walk gait_valid 6/6 both DR passes with
  ZERO sacrificed legs (the hard lock-in FAIL clause never fires) —
  watched det video confirms all six legs cycling, no flag leg. It
  falls via roll on every det episode (roll_class fell 6/6, tail
  ~4°, term tilt_roll) — the SAME campaign-wide universal takeoff-roll
  transient (CURRENT_TRUTHS), not new/transformer-specific, and not
  gated at this budget. fps ended 1651 (peaked 1887), short of the
  informational ≥2000 floor — same ad hoc-CUDA-torch softness as
  below, not a fail. **Per the pre-registered gate: PASS → respec 40M
  hardening twin, launched + VERIFIED same cycle as
  `cw-arch-tf-r1-hard1` on train-1 (reuses the proven ad hoc
  CUDA-torch build; fps estimate 8192 at launch — the durable
  launcher-level CUDA-torch path stays the open `[code]` item
  below).** Evidence: `rl_docs/runs/cw-arch-tf-r1b.md`.
- **08-15 ~12:1x UTC — `cw-arch-tf-r1` (transformer 2M discovery,
  train-0) checkup-SUSPECT resolved: slow but healthy, left running
  (finishes ~13:45 UTC).** The watcher's fps flag (1092 < 1875 floor)
  is NOT starvation — the run is solo with free cores and 0% GPU
  util. Root cause: torch on every mjx-train pod is the **CPU-only
  build** (2.13.0+cpu), so all SB3 policy nets have always trained on
  CPU; the hist16 MLP tolerates it (r7 ~4.7-5.6k fps) but the
  transformer trunk runs ~250-300 fps steady (~18x slower, batched
  impl verified clean — it's the build, not the code). The 2M rung
  still answers its behavioral question (boot/NaN/cheat lock-in);
  the gate's "fps usable >=2000" clause is answered NO for
  environmental reasons. **Named [code] item (blocks the
  pre-registered 40M hardening twin, 37h at current fps):** build an
  OPT-IN CUDA-torch path for the policy net (separate venv on one
  pod, launcher opt-in flag, bit-parity + GPU-memory-coexistence
  smoke vs warp) — shared default stays CPU-torch, no fleet-wide
  build swap. Ledger `checkup_note` on the run has the full chain.
  **UPDATE 08-15 ~12:2x UTC: superseded — the run was KILLED (no
  science read; env-only) once the fps math (37h for the pre-
  registered 40M twin) made waiting on it pointless, and relaunched
  as `cw-arch-tf-r1b` on train-1 with an ad hoc CUDA-torch install
  (`pip install torch==2.11.0+cu128 --no-deps`, kept train-1's
  existing jax/nvidia-cu12 stack intact) + `--device cuda`.
  Benchmarked ~120x on the identical PPO update (240s/iter CPU vs
  2.0s/iter cuda); `cw-arch-tf-r1b` verified RUNNING, fps climbing
  993→1457 over its first ~5 update cycles (still short of the
  gate's >=2000 floor at last look, trending toward it) and healthy
  (no NaN, EV climbing, coexists on train-1 fine alongside the
  still-running Arm A distill CPU job below). This is a ONE-POD,
  UNDOCUMENTED-BY-CODE fix — ephemeral (lost on pod restart/recycle,
  invisible to snapshot.sh's code marker) — so the named [code] item
  above (a real launcher-level opt-in CUDA-torch path: recorded pod
  capability, parity/coexistence smoke, reproducible install step)
  STAYS OPEN as the durable version of this fix; treat train-1's
  torch build as a manual, temporary exception until it lands.**
- **08-15 ~15:2x UTC — Arm A stage-0 distill VERIFIED: FAIL, no
  Stage 1 launch.** `ppo_goal_cw_arch_modeexperts_bc1.zip` (distilled
  from `footlow2_hard1` + `bcgait1_hard1`) does not match its
  teachers cold: det single-mode rise 0/6 (stalls 40-80mm short of
  full stand vs teacher's 0.5-3.4mm), det walk prog_ratio med 0.53
  with 2/6 episodes collapsing (prog 0.02/0.12, slip/m 26.2/7.7);
  hold/lower det clean (6/6 each). Sequence det 10/12 zero-fall
  (bar 11/12), sto 3/12. Per the pre-registered FAIL branch this is
  infrastructure (distill can't match teachers), not a science
  verdict — no PPO on this artifact. Leading fix candidate: the
  DAgger correction budget over-weighted lower's falls while rise
  stayed under-corrected (training probe returns already showed this:
  rise `['-217','125']` vs walk `['837','734']`) — a rise-targeted
  DAgger/coverage redesign is the next lever, left for a future
  cycle. Full numbers + evidence paths: `MODE_EXPERTS_DIRECTIVE.md`
  "Arm A" Stage 0.
  **08-15 ~17:0x UTC (drain-before-backoff cycle) — the redesign
  LANDED and re-collection is RUNNING.** `distill_gru --dagger-extra-
  mix/--dagger-extra-episodes` (default off, tests green, snapshot
  `exp/arch-modeexperts-bc2-rise-dagger`): a second single-mode
  targeted DAgger pass tops up rise's correction density independent
  of whether it triggers a hard fall (bc1's fall tally was almost
  entirely lower). `bc2` = bc1's exact recipe + `rise=1.0`/100 extra
  DAgger episodes per round, running as a CPU job on train-0
  (`/tmp/modeexperts_bc2.log`, `ppo_goal_cw_arch_modeexperts_bc2.zip`
  when done). Next cycle re-runs the same VERIFY before any Stage 1
  PPO; two-miss discipline applies (a second miss escalates
  `[operator]`, not a third variant). Detail:
  `MODE_EXPERTS_DIRECTIVE.md` "Arm A" Stage 0.
- **CROSS-TRACK INSIGHT (08-15, from hw `cw-stand-postlower3` dig-in,
  Cohort c3):** the shared mode-sequence rise branch
  (`_seq_segment_traj`) starts every mid-sequence rise at BELLY-FRAME
  0 with the blend interpolating the height ref DOWN from the current
  height — training on `goal.mode_seq`/`goal.mode_seq_stance` PAYS
  the policy to re-descend to belly and re-run the flat-rise
  choreography after a lower (hw measured: held-out det post-lower
  rise 0.967→0.419, over_current stalls mid-curl; detour visible in
  clean runs too). Any arch arm training with `goal.mode_seq` > 0
  (e.g. the modeexperts scratch line at 0.2) inherits this teacher.
  Fix exists, default-off: `goal.mode_seq_rise_from_h=1` (rise starts
  at current height; hw evidence pending Cohort c4 on
  `cw-stand-postlower4`). Adopting it in arch specs is an arch/
  operator call — no arch launch from the hw cycle. Details:
  `rl_docs/tracks/hw/SESSION_BULK_GATE.md` "Cohort c3 DIG-IN VERDICT".
- **08-15 ~06:2x UTC — Arm B canary `cw-arch-modeexperts-scratch1-r1`
  TRIAGED PASS (mechanism health, all four pre-registered clauses):
  finished 2.03M steps clean, no NaN/crash/canary-stop; FINAL
  experts/tick_frac_ rise=.345 loco=.260 lower=.217 hold=.178 vs
  commanded .35/.35/.20/.10 — every expert within 0.09, i.e. the mix
  self-corrected past the ±0.10 clause by end of run (better than the
  1.05M mid-run snapshot's hold=.246 miss, exactly as predicted:
  "as walking improves f_seq self-shifts toward walk"); per-expert
  std diverged independently from the shared .368 init (hold .393 >
  rise .389 > loco/lower .385); reward quarters −330.9/−308.1/
  −163.8/−2.4, monotone, no divergence. Skill acquisition explicitly
  NOT judged at 2M (mechanism-only gate). **PASS → `scratch2` (40M,
  corrected diet rise/loco/lower≈.30 each, hold≈.03,
  goal.mode_seq=0.2) LAUNCHED + VERIFIED same cycle on train-2** per
  the pre-registered SCRATCH2 order (fb_20260815T035147_dd2af0).
  Evidence: `rl_docs/runs/cw-arch-modeexperts-scratch1-r1.md`.
- **08-15 — OPERATOR DIRECTIVE EXECUTED (fb_20260815T013349_488ffd,
  via operator KICK): the four-expert isolated architecture is BUILT
  and both arms are pre-registered — full spec + gates + decision
  table in `MODE_EXPERTS_DIRECTIVE.md`.**
  `ModeExpertsGruActorCriticPolicy` (rise/hold/lower/loco experts,
  each its own actor GRU + critic GRU + heads + PER-EXPERT log_std —
  the two sharing channels dual2/modeseq1-r1 died through are gone by
  construction; optional zero-init transition adapter; freeze
  support), default-off, legacy bit-exact, tests + semantics bank
  green, snapshotted. **Arm A** `cw-arch-modeexperts1` (composition):
  stage-0 distill from footlow2_hard1 (stance experts) +
  bcgait1_hard1 (loco) RUNNING on train-1 CPUs; stage-1 frozen-expert
  adapter PPO pre-registered `[precondition: distill verifies vs
  teachers]`. **Arm B** `cw-arch-modeexperts-scratch1` (from-scratch
  WALK+RISE+LOWER, operator override of the nobc gait closure; no BC
  anchor, no rise_ref imitation): 2M mechanism CANARY
  `cw-arch-modeexperts-scratch1-r1` RUNNING on train-2 (first launch
  died silently on train-0 — INFRA, verdicted, retry verified);
  staged ~60M full-budget acquisition pre-registered on canary PASS —
  "not learned at 2M" is explicitly NOT a verdict (budget honesty:
  cw-mt-a2 needed 20M active walk ticks). Multitask pause explicitly
  lifted for these two arms only; hierarchy product baseline
  untouched. **08-15 ~04:1x UTC (overnight directive
  fb_20260815T035147_dd2af0 executed): scratch2 is fully pre-staged.**
  Canary healthy at 1.05M (reward −301→−238, independent per-expert
  stds, ~195 fps) but the realized skill diet misses the tick clause:
  hold gets .246 of ticks (ordered .10), loco .234 (ordered .35).
  Measured attribution: sequence episodes deliver f_seq = rise .29 /
  loco .12 / lower .20 / hold .39 (rise→hold precedes walk in the
  grammar; early falls truncate walk tails) — operator pre-classified
  this exact miss as a curriculum SPEC DEFECT, not an architecture
  verdict. Corrected phase-1 curriculum solved from measured
  fractions (mode_seq 0.5→0.2, mix walk=.345/rise=.303/lower=.324/
  hold=.028 → predicted realized .30/.30/.30/.10, preflight PASSED);
  exact respec command, exposure-honesty gate, watchdog/checkpoint/
  canary-stop safeguards, disclosed limitations, and the scratch3
  active-tick top-up (≥20M REAL ticks/skill; never report env steps
  as exposure) are pre-registered in MODE_EXPERTS_DIRECTIVE.md
  "SCRATCH2". The cycle that triages the canary launches scratch2
  SAME CYCLE.
- **08-14 ~19:5x UTC — `transdagger3` TRIAGED: FAIL on the Arm 1
  gate, NET REGRESSION vs transdagger2 on the sequence clause — the
  rise demo mix is ZERO-SUM.** Seq det DR0 zero-fall 9/12 (bar
  ≥11/12; td2 12/12; sto 10/12 = td2): the bridge/flat-heavy mix
  fixed its target (cold first rise 12/12 in-seq vs td2's 5/12) but
  all 3 det falls moved into the post-lower rise (td2's strongest
  segment), and rise12 retention got WORSE (2/12 all-crouch vs td2's
  3/12; bar ≥3/12 with ≥1 non-crouch). Lower/hold/walk retention all
  held (lower 6/6+6/6 worst_clear 0mm; walk gv 21/21 in-seq + 6/6).
  Second data-mix miss → mechanism change per two-miss; no
  pre-registered FAIL branch matches → no transdagger4.
  **transdagger2 stays the winning distill artifact/init.** Open rise
  levers are both operator-routed: option (b) rise-only-DAgger
  variant (spec sharpened: ADD rise coverage across ALL start kinds,
  don't re-weight a fixed budget) and/or the anchor-on-rise Arm-2
  retry mechanism. Full scorecard: TRANSITIONS_DIRECTIVE
  "TRANSDAGGER3 RESULT".
- **08-14 ~17:xx UTC — Arm 2 `cw-arch-modeseq1-r1` VERDICTED FAIL
  (dig-in): the canary auto-stop at 4.56M was dual2's protected-skill
  erosion, second independent reproduction — and training 75% on
  sequences did NOT protect the demo rise.** Sequence gate det DR0
  zero-fall **2/12** (bar ≥11/12; sto 3/12) vs transdagger2's 12/12
  and the specialist baseline's 11/12; every fall is INSIDE a rise
  segment (all 3 flat cold-starts fell, 5/7 post-lower rises fell;
  switches clean ≤8.2°; walk segments 9/9 gait_valid prog 1.014,
  lower 7/7). Matched rise recheck (n=12/seed=1 DR0): det 5/12 =
  crouch 5/5 / bridge 0/4 / flat 0/3 — the EXACT dual2 3.18M endpoint
  profile vs the dagger1-init control's 3/12 all-non-crouch; and
  flat/bridge attempts now FALL instead of stalling. Single-mode
  retention letter-passes (walk gv 6/6 prog 1.04, hold 6/6, lower 6/6
  worst_clear 0mm). None of the three pre-registered Arm-2 FAIL
  branches matches. **Ruling: warm-RL from the dagger1 init is CLOSED
  per the two-miss rule (dual2 + this); the sequence-diet hypothesis
  is refuted — PPO+stance-anchors spend rare hard-start demo
  competence regardless of diet.** Per the directive's pre-named
  fork, the agent-doable half was EXECUTED same cycle:
  **`transdagger3` — the transdagger2 recipe with a bridge/flat-heavy
  demo mix (`--cfg-set goal.rise_flat_frac=0.45
  goal.rise_partial_frac=0.45`, crouch 0.10; passthrough flag landed
  this cycle, default-off/bit-exact) — ran on train-0 CPUs (TRIAGED
  FAIL 08-14 ~19:5x, see the bullet above)**;
  the operator's option (b) rise-only variant distill stays open in
  top-level WAITING-ON. An advisory external-feedback note
  (fb_20260814T164337_d7f11b: add a bridge/flat-rise BC-anchor term
  to the eventual Arm-2 retry, not just diet) is recorded in
  TRANSITIONS_DIRECTIVE under "ARM 2 RESULT" for the retry spec.
  Detail: `rl_docs/runs/cw-arch-modeseq1-r1.md`;
  evidence `logs/ckpt_eval/modeseq1_r1_seq_{det,sto}.json`,
  `cw_arch_modeseq1_r1_{gate,owncfg,rise12}`.
- **08-14 ~12:4x UTC — transdagger2 TRIAGED (the named next step):
  FAIL by the letter of the Arm 1 gate, but only on the two rise
  clauses — the sequence-fall problem itself is SOLVED in one model
  for the first time.** `ppo_goal_cw_gru_dual_bc_transdagger2.zip`
  scores **12/12 det zero-fall** on the sequence eval (above the
  two-specialist baseline's 11/12; sto 10/12 vs 9/12), walk segments
  24/24 gait_valid, lower segments 12/12, **single-mode lower REBUILT
  6/6 det+sto worst_clear 0mm** (dagger1: 0/6, 261mm airborne
  tripod), hold 6/6, switch tilt ≤7.7°; video honest. The misses:
  cold FIRST rise 5/12 det (ends 4–17mm short, zero falls) and the
  rise12 retention recheck 3/12 with **0 non-crouch wins** (init had
  3) — the DAgger rounds traded bridge/flat rise finish for
  crouch/plant competence; no pre-registered FAIL branch matches, so
  no discretionary transdagger3. Full scorecard in
  TRANSITIONS_DIRECTIVE "ARM 1 RE-RUN RESULT". **Consequence: Arm 2
  `cw-arch-modeseq1` QUEUED same cycle** per the no-discretion
  warm-start order → init (3) `ppo_goal_cw_gru_dual_bc_dagger1.zip`
  (gates (1) and (2) both FAIL), dual2's exact stack + episode 30s +
  `goal.mode_seq=0.75` — the 75/25 sequence/single-mode diet hook
  (fractional mode_seq) was the one missing cfg and LANDED this
  cycle (snapshot 2ef85f7, endpoints bit-exact, mode_seq bank 11/11
  + semantics bank green).
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
  operator-gated.** Progress:
  - **CODE item 1 LANDED (08-13 ~22:xx, c-modeseq-code):
    `goal.mode_seq`** (walk_task/sim_env, per the scoping note below
    — the goal-derived half of the episode state re-derives at each
    switch, physical reset untouched). K-segment episodes on the
    grammar rise→{hold|walk}→{walk|lower}→(rise…); at each switch the
    height frame re-anchors (`_z0` = the CURRENT chassis height —
    directly attacks lesson 5 and the baseline's second-rise
    weakness below), refs regenerate on the episode clock with an
    absolute-continuous blend window (lesson 6), BC-anchor
    eligibility flips with the segment and hold/lower anchors
    re-base on the pose carried INTO the segment (never the reset
    q_nom), walk income accumulators restart per segment, the
    end-posture window clamps to the segment. Default 0 = bit-exact
    legacy (no extra rng draws); 5 new per-episode attrs ride
    `mjx_host.SNAP_ATTRS`. Tests: `rl_move/tests/test_mode_seq.py`
    (8: off-by-default stream equality, grammar/start-kind
    compatibility, lower→rise re-anchor, blend continuity, one-hot
    flip, walk-state restart, SNAP membership) + full regression
    green (semantics bank 91 pass/1 xfail; sim_env, mode_onehot,
    bc_anchor, gru_policy, mjx_vec_env suites all pass).
  - **CODE item 3 LANDED + BASELINED** (same day, c-triage — see the
    dedicated bullet below; baseline: footlow2_hard1 composition
    11/12 det, 9/12 sto, all sto falls on the post-lower rise).
  - **CODE item 2 LANDED (08-13, c-idlekick post-dual2): `distill_gru
    --transitions`** — sequence demos ride item 1's `goal.mode_seq=1`
    env (env chains + re-anchors; per-tick teacher routing by active
    segment; one continuous stream, one-hot flips; hard in-context
    teacher-verification abort; sequence DAgger labels every segment
    incl. lower). Default 0 = off/bit-exact; tests
    `test_distill_transitions.py` + full regression green; tiny
    end-to-end smoke clean (0 teacher falls). Details/landed-note in
    TRANSITIONS_DIRECTIVE.md item 2.
  - **Arm 1 `cw-arch-trans-dagger1` LAUNCHED (08-13, same cycle):
    CPU job on hexapod-mjx-train-0's idle cores (fleet 0/12 GPU) —
    300 seq eps (30 s) + 200 single-mode retention eps (stance-heavy
    mix), 30 epochs, 2 DAgger rounds x 100 seq eps, teachers
    walk_longdist_r2 + stance_dr10 (directive default), artifact
    `ppo_goal_cw_gru_dual_bc_transdagger1.zip`. Gate = the directive's
    pre-registered Arm 1 gate (sequence eval det DR0 zero-fall >=11/12
    + per-segment >=8/12 + retention: hold >=5/6, walk gv >=5/6, rise
    n=12/seed=1 >=3/12 with >=1 non-crouch, lower REBUILD >=4/6).
    NOT a ledger/GPU run (dynrep precedent) — log
    `/tmp/transdagger1.log` on train-0; next cycle triages the
    artifact through `eval_modeseq` + the harness, then specs arm 2
    (`cw-arch-modeseq1`; warm-start order trans-dagger1 > dual2 >
    dagger1 BC — note dual2's slot is a 3.18M mid-transient ckpt,
    re-judge per the dual2 verdict).
  - **Gate instrument made Arm-1-ready (08-13 ~23:xx, this cycle,
    while the CPU job ran its BC epochs): `eval_modeseq --single`**
    — one checkpoint drives every segment, GRU hidden state carried
    ACROSS switches (the `--transitions` continuous-stream
    contract), dual-core one-hot auto-detected; walk segments now
    gate on gait_valid per segment (lesson 11) and every segment
    reports switch-window tilt/current. v1 could not evaluate a
    single-model artifact at all — the Arm 1 gate would have been
    unrunnable at triage time. Smokes on train-1: specialist path
    reproduces the baseline in-band; `--single` on the dagger1 zip
    reproduces its known skill profile. Baseline det+sto refreshed
    with the new fields (`modeseq_baseline_footlow2_{det,sto}_v2
    .json`). Job health at check: BC RMS 0.1552 (better than
    dagger1's 0.1865), DAgger rounds in progress, 0 verify falls.
    Details: TRANSITIONS_DIRECTIVE.md item 3 GATE-READY note.
  - **Arm 1 first artifact VERDICTED FAIL (08-14) — but the
    `--transitions` mechanism is EXONERATED and the fix is one flag,
    already running.** transdagger1 on the sequence gate: det+sto
    **0/12 zero-fall** (bar ≥11/12); walk segments 12/12 gait_valid
    (the rise→walk switch transfers), but lower parks a whole tripod
    (legs 0/2/4, 80–260mm in air) and the post-lower rise tips
    tilt_roll 12/12. **Matched-teacher control: the default stance
    teacher `stance_dr10` scores 0/12 det on the same instrument with
    the identical per-segment fingerprint** (and fell 30/300 demo
    sequences during collection — rise 24, lower 5) — the student is
    a high-fidelity copy of an in-context-incapable teacher, exactly
    CURRENT_TRUTHS' "anchors can TEACH a defect". The falls-only
    `--seq-verify` missed it (tripod-up lower doesn't fall). Retention:
    hold det 6/6 + walk gv 6/6 + rise n=12/seed=1 det 3/12 with 3
    non-crouch (= dagger1 init) all PASS; **lower 0/6 NOT rebuilt —
    and in the teacher's tripod-up shape (worst_clear 261mm), not
    dagger1's belly-drag collapse: the defect was TAUGHT, in and out
    of sequence.** The one-flag fix `cw-arch-trans-dagger2`
    (`--stance-teacher` → `footlow2_hard1`) was launched AND KILLED
    the same cycle — the kill is the bigger finding: **in the
    `goal.mode_seq` training env that teacher pair fell 99/225 demo
    sequences (lower 73) while scoring 11/12 zero-fall on the eval
    instrument; stance_dr10 shows the INVERTED pattern (30/300
    in-env, 0/12 instrument). CODE item 1's in-env per-switch
    re-anchor (walk→lower especially) does not reproduce the proven
    handoff context and is now the prime suspect — debug it before
    ANY re-distill, and arm 2 must not train on `goal.mode_seq=1`
    until fixed** (full plan in TRANSITIONS_DIRECTIVE.md "ARM 1
    RESULT"). Arm 2 also stays off the slot-3 (dagger1 BC) init
    while the slot-1 repair is in flight (ASSUMPTION, operator to
    review). Evidence:
    `logs/ckpt_eval/transdagger1_modeseq_{det,sto}.json`,
    `modeseq_stancedr10_det.json`,
    `cw_arch_trans_dagger1_{gate,rise12}` (all train-0); full note in
    TRANSITIONS_DIRECTIVE.md "ARM 1 RESULT".
  - **CODE item 1 re-anchor bug FOUND, FIXED and RE-VERIFIED (08-14
    ~02:xx UTC, operator session, local Mac) — the blocker on both
    arms is CLEARED for the CPU path.** Root cause (instrumented
    diff, exactly the pre-registered next step): the v1 in-env switch
    carried the EPISODE-reset `q_nom` across segments ("flip nothing
    else") and re-based `_z0` on the instantaneous chassis height,
    while the proven `reanchor_to()` derives `q_nom`/`_z0`/pad refs
    from a fresh settled reset of the TARGET mode. Obs joints are
    `(q − q_nom)`: the settled belly-flat vs settled-plant `q_nom`
    differ by **78.9° at the knees** (measured), so every rise-start
    sequence fed all later plant-family segments a belly frame that
    far off the teachers' training distribution — which is why LOWER
    fell hardest (73/225: lower is never a first segment) and why the
    eval instrument (canonical frames by construction) contradicted
    the env. Fix (`sim_env.py`): reset()-time settle probe
    `_seq_capture_frames` mints the canonical plant+belly frames on
    the episode's OWN model (exact reset choreography; cached when
    the model can't change, re-minted per episode under DR);
    `_seq_maybe_switch` installs the target family's frame
    (`q_nom`/`_z0`/`_pad_z_ref`) at every switch; hold/lower BC base
    is now that canonical `q_nom` (the v1 carried-pose anchor is
    dropped — its belly-trap rationale is gone and the teachers'
    own base is the plant frame); physics, servo profile, slew
    memory and tilt frame still carry over (they are physical);
    `_seq_frames` rides SNAP_ATTRS. Guard: the MJX batched-reset
    path never runs env.reset(), so mode_seq there raises loudly —
    **a batched frame mint is the named follow-up CODE item before
    arm 2 can train on MJX.** Tests: frame parity locked as a
    regression test (probe frames == fresh-reset frames, the
    reanchor_to() equivalence), lower→rise switch asserts the belly
    frame install; mode_seq suite 9 pass, full local bank 311
    pass/1 xfail. **Re-verify (pre-registered condition, exact
    collection context — DR0.5, 30 s eps, stance-heavy first mix,
    seed 0, `verify_modeseq_teachers.py`): footlow2_hard1 +
    walk_longdist_r2 fall 12/225 (5.3%; lower 5, walk 3, rise 2,
    hold 2; det 9/163) with teacher return med 658 min −79 — vs
    99/225 (44%, lower 73, med ~298 min −485) on the pods pre-fix,
    AND a same-machine/same-seed pre-fix A/B (worktree at the old
    code, identical driver) that replicates the pod number at
    92/225 (40.9%, lower 62, med 287 min −560): the fix, and only
    the fix, moves 41%→5.3%. ≈ the instrument's own det band
    (11/12 zero-fall).** stance_dr10
    control: 9/300 (3.0%) vs 30/300 pre-fix — the inverted pattern
    is gone; its tripod-up-lower QUALITY defect remains (and remains
    invisible to falls-only counting: the ledger's teacher-quality-
    gate lesson stands, stance_dr10 stays disqualified). Artifacts:
    `rl_move/sim/logs/verify_modeseq_{footlow2,stancedr10}_postfix
    .json` (local). **Next: re-run the transdagger2 recipe
    (`--stance-teacher footlow2_hard1`, CPU) on the fixed env; arm 2
    stays held only on the MJX frame mint + the transdagger2
    result.**
  - **BOTH next steps EXECUTED (08-14 ~03:xx UTC, orchestrator
    idle-kick): (1) the transdagger2 recipe is RUNNING on train-0
    CPUs** (fixed env synced to the pod first; in-context teacher
    verify PASSED on-pod — 2/12 det falls, cap 4, teacher return med
    656 ≈ local 658; collection 500 eps at ~4% falls, BC epochs
    running; artifact `ppo_goal_cw_gru_dual_bc_transdagger2.zip`,
    log `/tmp/transdagger2.log`, gate = the directive's Arm 1 gate
    unchanged). **(2) The MJX batched frame mint LANDED — arm 2's
    named CODE wait is CLEARED** (`MjxVecEnv._mint_seq_frames`,
    commit 8374125 / tag exp/mjx-modeseq-mint1): the batched
    choreography mints canonical plant/belly frames per episode
    (device twin of `_seq_capture_frames`, re-minted under DR),
    default-off/bit-exact when mode_seq=0. Pod-verified on train-1
    CPU MJX: frame parity vs a fresh C reset within 0.03 rad/6 mm
    (`test_mode_seq_frames_minted_and_match_c_env`), batched
    episodes now cross switches (`test_mode_seq_switch_crosses_on_
    batched_path`; pre-mint = RuntimeError). Caveat logged: the
    pre-existing `test_sharded_bitwise_matches_inprocess` failure
    on train-1 reproduces at unmodified HEAD (~1e-5 sharded vs
    in-process obs drift) — pod-env issue, NOT the mint; flag to
    the next MJX dig-in. Arm 2 now waits only on the transdagger2
    triage.
  The directive's failure ledger (12 measured lessons) is binding —
  do not re-litigate closed levers inside these arms.
- **`cw-arch-gru-dual2` VERDICTED FAIL (08-13 ~23:xx UTC dig-in) —
  the operator's option (a) fork is resolved: DAgger-init rise
  gains do NOT survive warm-RL; option (b), the rise-only variant
  distill (operator/local), is now the required next lever (WAIT
  entry added to top-level STATUS.md).** The canary auto-stop at
  3.18M was the system's FIRST TRUE protected-skill catch: the
  DAgger init passed rise_bridge 2/2 at the baseline probe (first
  init in the lineage to do so — dual1's init failed it, so dual1
  was never bridge-protected and its all-zero bridge canaries mean
  nothing), and every probe from 1M on read 0/2. Pre-registered
  n=12/seed=1 DR0 rise recheck on the stopped checkpoint: **det
  5/12 with ZERO non-crouch (bridge 0/4, flat 0/3, crouch 5/5)**
  vs the matched control (dagger1 init, same seed/cfg, same pod:
  det 3/12 ALL non-crouch — bridge 2/4, flat 1/3, crouch 0/5 —
  reproducing the operator's local numbers exactly). RL SWAPPED
  the rise profile back to the crouch attractor (hfloor1's exact
  endpoint) within 1M steps: PPO+stance-anchors spend the rare
  hard-start demo competence first. More budget is unjustified
  (gone by 1M; dual1's full 10M from scratch reached only 2
  non-crouch wins) and banned by the pre-registration. Videos:
  honest low-crouch stalls, no cheat. **Two findings shape option
  (b): (1) lower REBUILT det+sto 6/6 from the COLLAPSED (0/6)
  init — the 'keep lower BC-only' requirement is RELAXED; (2) walk
  did not freeze (gv 6/6, zero sacrificed legs; prog 0.61/vel_err
  0.046 is dual1's own 3.2M mid-run transient, not an isolation
  regression — escalate branch does not fire).** NOTE for arm 2's
  pre-registered warm-start order (trans-dagger1 > dual2 > dagger1
  BC): dual2's artifact is a 3.18M mid-transient checkpoint whose
  walk is WORSE than the dagger1 BC init's (0.61/0.046 vs
  1.03/0.034) and whose rise is crouch-only — re-judge that slot
  when arm 2 specs. Evidence:
  `logs/ckpt_eval/cw_arch_gru_dual2_{gate,owncfg,rise12}` +
  `gru_dual_bc_dagger1_rise12_pod` (control); full chain in
  rl_docs/runs/cw-arch-gru-dual2.md.
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
