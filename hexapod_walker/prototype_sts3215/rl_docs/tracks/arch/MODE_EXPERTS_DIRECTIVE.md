# MODE-EXPERTS DIRECTIVE — four fully isolated experts, one checkpoint

Operator directive (Lukas, 08-15, delivered via operator KICK focus
note executing feedback `fb_20260815T013349_488ffd`; builds on the
W&B retrospective `fb_20260814T211335_df648b`). EXPLICITLY authorized
despite the 08-13 multitask pause and, for Arm B, despite the nobc
from-scratch-gait closure — operator override, sim only, research
arms. The measured hierarchy stays the product baseline and is NOT
modified by anything here.

**Plain English:** every one-model attempt so far shared parameters
between skills — first one trunk (walk froze), then one stance core +
one global exploration std (PPO spent rare hard-start rise competence
through exactly those shared parts: dual2, modeseq1-r1). This
directive tests whether ONE checkpoint holds all skills when NOTHING
is shared: four experts (RISE / HOLD / LOWER / LOCOMOTION), each with
its own actor GRU, critic GRU, heads, and its own learnable log_std,
routed exactly by the mode one-hot; gradients cannot cross experts by
construction.

## Code (landed 08-15, this cycle, default-off)

- `gru_policy.ModeExpertsGruActorCriticPolicy` (+ `_QuadGRU`,
  `EXPERTS_ORDER=(rise,hold,lower,loco)`): complete per-expert
  isolation incl. per-expert log_std; all four memories stay warm
  every tick, only the active expert's output/gradient is selected;
  optional transition adapter (`experts_adapter_hidden>0`) = small
  residual MLP on the selected mean, ZERO-init (exactly 0 at init),
  architecturally separate from expert bodies;
  `set_experts_frozen()` freezes expert actor bodies (cores, actor
  latents, heads, log_stds) leaving adapter + critics trainable.
- `train_ppo_mjx`: `--gru-experts`, `--gru-experts-adapter N`,
  `--gru-experts-adapter-scale`, `--gru-experts-freeze`; logs
  per-expert ACTIVE ticks + tick fractions (`experts/active_ticks_*`,
  `experts/tick_frac_*`) and per-expert std (`experts/std_*`).
- `distill_gru`: `--experts` (+ `--experts-adapter N`, adapter frozen
  during BC/DAgger) — one BC pass distills all four experts because
  routing sends each tick's gradient only to its own expert.
- Tests (`test_gru_policy.py` §6): routing (mean AND std), zero
  actor/critic/log_std gradient into inactive experts, adapter
  zero-init bit-exactness + freeze isolation, deterministic save/load
  roundtrip, bptt/anchor path parity; legacy classes untouched (full
  suite green). Semantics banks green (91 passed; only quadwalk
  skips, irrelevant here).
- NOT implemented (disclosed limitation): per-mode/start-kind PPO
  MINIBATCH stratification quotas. Exposure control is env-level
  (goal-mix, mode_seq fraction, rise start-kind fracs) and active
  ticks are measured and reported per expert. Adam moments are
  per-parameter, so optimizer state is per-expert-isolated by
  construction; one optimizer object serializes all of it.

## Arm A — `cw-arch-modeexperts1` (pretrained modular composition)

Question: does one checkpoint PRESERVE the working skills and learn
only their boundaries once destructive shared gradients are removed?

- Stage 0 (RUNNING 08-15, train-1 CPUs): distill
  rise/hold/lower experts from `footlow2_hard1` and loco from
  `bcgait1_hard1` — `distill_gru --experts --experts-adapter 32
  --transitions 300 --episodes 200 --dagger-rounds 2` (transdagger2's
  proven recipe, new teachers per the directive) →
  `ppo_goal_cw_arch_modeexperts_bc1.zip`. In-context teacher
  verification (12 det sequences, abort >4 falls) guards a
  teacher pair that cannot chain. VERIFY before stage 1: eval the
  distill vs each teacher (single-mode det: rise/hold/lower vs
  footlow2_hard1 bars, walk gait_valid vs bcgait1_hard1) + sequence
  eval vs the c1 hierarchy baseline. A broken distill is
  infrastructure, not science — fix or re-collect, don't launch.
- Stage 1 (pre-registered, [precondition: stage-0 artifact passes
  verification]): `cw-arch-modeexperts1` — 2M discovery PPO, warm
  from the distill, `--gru-experts --gru-experts-freeze` (expert
  bodies frozen; trains ONLY the transition adapter + critics) on
  goal.mode_seq sessions (0.75 sequence / 0.25 single-mode,
  stance-heavy first-mix, full start-kind diversity). Gate
  (pre-registered): sequence det DR0 zero-fall >= its own distill
  init AND >= 11/12, retention EXACTLY at distill level (frozen
  experts cannot erode — any retention drop is an implementation bug
  and FAILS the arm), adapter effect reported (switch-window tilt vs
  init). Judged additionally on the c3-style fresh bulk banks
  (300 det + 300 sto) vs c1 hierarchy / td2 / dual1 / dual2 /
  modeseq1-r1 before any promotion claim.
- Stage 2 (only after stage-1 improvement is demonstrated):
  low-rate joint unfreeze, per-expert canaries, pre-erosion
  checkpoints. Separate spec, not authorized to launch yet.
- FAIL branches (pre-registered): stage-0 distill can't chain /
  can't match teachers → infrastructure, fix distill, no PPO;
  stage-1 retention drops → implementation bug (frozen bodies), fix
  code; stage-1 transitions don't improve → adapter mechanism
  insufficient, escalate to operator before stage 2.

## Arm B — `cw-arch-modeexperts-scratch1` (genuinely from scratch)

Question (Lukas's hypothesis, direct test): can a sufficiently large
STRUCTURALLY ISOLATED model learn WALK + RISE + LOWER from random
weights in one training program?

- Same architecture/obs/action/session contracts as Arm A. Random
  init. NO checkpoint init, NO teacher targets, NO BC/DAgger/BC
  anchor, NO scripted-policy action loss. Explicitly EXCLUDED from
  the training cfg vs the modeseq recipe: every `train.bc_anchor_*`
  key AND `reward.rise_ref_track` (the scripted rise reference is
  demo-derived — using it would be hidden imitation). Curricula
  disclosed: the validated env reward stack (anti-cheat terms:
  step_event, drag_loaded, park_duty, kernel_prog_gate, walk anchor
  no-skate gate), goal-mix walk=0.35,rise=0.35,lower=0.20,hold=0.10
  (hold kept small for session settling; exposure reported
  separately via experts/tick_frac_*), 50% mode_seq sessions.
- Stage 1 (RUNNING 08-15 as `cw-arch-modeexperts-scratch1-r1` on
  train-2; the original name's launch died silently on train-0 —
  INFRA FAIL, verdicted in its ledger entry): 2M discovery CANARY.
  Gate is MECHANISM HEALTH ONLY: no crash/NaN, canaries quiet,
  routing live (all four experts receive ticks; tick fractions match
  the mix), per-expert std/grad curves move independently, active
  ticks reported. "Has not learned at 2M" is EXPLICITLY NOT a FAIL
  and NOT a capacity verdict (budget honesty: cw-mt-a2 needed 20M
  active walk ticks / ~306 updates).
- Stages 2+ (pre-registered, [precondition: stage-1 mechanism
  PASS]): staged full-budget acquisition targeting ~20M ACTIVE ticks
  per required skill (~60M total at this mix), as chained 40M-cap
  runs warm-started from each other (`-scratch2` 40M, `-scratch3`
  +20-40M), checkpoints evaluated at ~2M/10M/20M active ticks per
  skill. Phase mechanics: these launch as `--phase hardening
  --evidence "operator directive fb_20260815T013349_488ffd
  pre-authorizes the staged full-budget from-scratch test; from-
  scratch walking at this budget seen in cw-mt-a2"` — recorded here
  as an ASSUMPTION (operator to review): the phase system has no
  'authorized long-horizon acquisition' phase; the operator's budget
  order supersedes the 2M discovery cap for this line only.
- Read-out: per-expert curves name exactly which skill failed
  (isolation ⇒ "cannot acquire rise even alone" is distinguishable
  from interference). If all three acquired without exploit: two
  seed twins before any success claim. Eval on fresh retired-after-
  read bulk banks, isolated + session strata, vs Arm A and the
  hierarchy.

## SCRATCH2 — pre-registered launch order (08-15 ~04:1x UTC, overnight
## directive fb_20260815T035147_dd2af0; execute in the cycle that
## triages the canary)

**Canary telemetry at 1.05M (W&B, live read):** mechanically healthy —
reward −301→−238, no NaN/crash, all four experts active, per-expert
stds diverging independently (hold .387 / rise .380 / lower .379 /
loco .375), ~195 fps wall-average. **Realized active-tick fractions
are STABLE and MISS the gate's ±0.10-of-commanded clause:** rise .319
/ loco .234 / lower .202 / **hold .246** (commanded .35/.35/.20/.10).
Attribution (measured, not guessed): solving
`realized = 0.5·mix + 0.5·f_seq` gives the sequence-episode-only
fractions **f_seq = rise .29 / loco .12 / lower .20 / hold .39** —
the grammar puts `rise→hold` BEFORE any walk segment and a
from-scratch policy's falls truncate episodes at the walk segment, so
HOLD banks its ticks and WALK loses its tail. This is a
curriculum/spec defect (the operator pre-classified exactly this
case), NOT evidence against the architecture or the from-scratch
hypothesis. As walking improves f_seq self-shifts toward walk — hence
measured-truth monitoring + re-solve at each stage below.

**Triage checklist for the clearing cycle (mechanism health ONLY):**
(1) W&B state finished at 2M, no NaN/crash/canary-stop; (2) all four
`experts/active_ticks_*` > 0 and fractions ≈ the 1.05M read; (3)
`experts/std_*` still independent; (4) reward trend not diverging.
Tick-balance clause miss alone = **spec defect → PASS-with-defect →
launch scratch2 in the SAME cycle** (operator order; do not wait for
morning). Anything else broken = infra/mechanism triage per the Arm B
stage-1 branches. No behavior eval needed at 2M; no bulk cohort
(dd2af0: bulk only at forks/final candidates).

**Corrected curriculum (bounded preflight PASSED this cycle):** target
realized ≈ rise .30 / loco .30 / lower .30 / hold ≤.10 while keeping
complete sequences. Phase 1 = acquisition-heavy: `goal.mode_seq`
0.5→**0.2** and single-mode mix re-solved from the MEASURED f_seq:
`m = (target − 0.2·f_seq)/0.8` → **walk=0.345, rise=0.303,
lower=0.324, hold=0.028** (sums 1.000; trainer parser verified;
predicted realized .300/.299/.300/.101). Start-kind coverage is
preserved by construction (single-mode rise keeps its flat/bridge/
crouch mix; sequences keep legacy first-segment samplers and
lower→rise post-lower rises).

**Launch command (verbatim; train-2 keeps the canary ckpt local):**

    python3 rl_move/orchestrator/launch_run.py respec \
      --from cw-arch-modeexperts-scratch1-r1 \
      --run cw-arch-modeexperts-scratch2 \
      --steps 40000000 --init-from-source \
      --phase hardening \
      --evidence "Operator directives fb_20260815T013349_488ffd + fb_20260815T035147_dd2af0 pre-authorize the staged full-budget from-scratch acquisition stage (budget honesty: cw-mt-a2 needed ~20M active walk ticks); canary cw-arch-modeexperts-scratch1-r1 completed 2M mechanism-healthy (all four experts active, independent per-expert stds, reward improving, no NaN/crash); tick-imbalance clause pre-classified as curriculum spec defect and corrected here" \
      --arg='--goal-mix=walk=0.345,rise=0.303,lower=0.324,hold=0.028' \
      --arg='--canary-stop-after=0' \
      --cfg goal.mode_seq=0.2 \
      --hypothesis "Give the four-specialist from-scratch brain its real training budget with the skill diet the operator actually ordered: the 2M canary proved the mechanism healthy but measured that sequence episodes overfeed HOLD (25% of real practice ticks vs 10% ordered) and starve WALKING (23% vs 35%), because holds come before walks in the sequence grammar and early falls cut walks short. This 40M stage continues the same random-init lineage (no imitation anywhere) with a corrected diet — sequence episodes 50%->20%, single-mode mix re-solved from MEASURED sequence tick fractions — so stand-up, walking and sit-down each get ~30% of real practice ticks (~12M each this stage). If the isolated-experts architecture can acquire the skills, per-mode eval scores move within this stage; a skill that stays flat under full measured exposure indicts its reward/start curriculum, not capacity or interference." \
      --gate "ACQUISITION stage (pre-registered, MODE_EXPERTS_DIRECTIVE.md SCRATCH2): (a) completes 40M with no NaN/crash — silent death gets ONE retry from the latest periodic checkpoint (save-every 1M) on a clean pod, infra not science; (b) EXPOSURE clause: realized experts/tick_frac_* within ±0.05 of rise/loco/lower .30 each and hold ≤.15 at 10M and at end — a drift re-solves the scratch3 mix from measured f_seq, never a kill; (c) per-expert learning signal visible (independent stds + per-mode background C-env eval trends; aggregate return is NOT evidence). NO early stop for poor skill at any milestone — stop only for NaN/crash, a proven exploit dominating a milestone video, or clause (b) instrumentation dying. Skill success is judged only at fork/final via bulk cohorts (fb_20260815T033634_7d750e); 40M total is NOT the requested budget — scratch3 (pre-registered below) tops REAL active ticks to ~20M/skill." \
      --now --pod hexapod-mjx-train-2

**Safeguards already in place (verified this cycle, no new code):**
the checkup watchdog uses PID + CPU-time proof-of-work before calling
a stall (launch_run.py `_pod_pid_cputime`, added after a false
SUSPECT on this exact canary — the 6-min recurrent update cadence
cannot fool it); periodic checkpoints every 1M steps are the trainer
default (`--save-every`); SB3 `.load` restores optimizer state and
the per-expert log_stds (ModeExperts save/load roundtrip is
test-locked); mode-switch re-anchoring semantics unchanged (mode_seq
frame mint is the tested 08-14 code). `--canary-stop-after=0`
(monitor only) because warm-start auto-enables regression canaries
and a near-zero-skill baseline makes auto-stop pure false-kill risk —
exactly dd2af0's "do not early-stop because behavior is still poor".
Wall clock disclosure: ~195 fps → 40M ≈ 2.3 days on train-2.

**Disclosed limitations (dd2af0 asks, not built tonight):** per-mode
PPO minibatch stratification quotas and per-expert actor/value-loss/
KL/grad-norm decomposition are NOT implemented — building them would
touch the shared PPO train loop hours before an operator-ordered
launch. Compensation: env-level measured-mix correction (above),
per-expert active-ticks/fraction/std logging (live), and the per-mode
background C-env eval SCORE curves as per-expert task proxies; no
aggregate-return claim is admissible for any single expert.

**SCRATCH3 (pre-registered, launches only after scratch2 triage):**
+20–40M warm from scratch2, mix re-solved from scratch2's measured
f_seq so that CUMULATIVE lineage active ticks reach ≥20M per
rise/loco/lower (never report total env steps as skill exposure;
canary contributed only ~0.6/0.5/0.4M). If per-skill acquisition has
appeared, raise `goal.mode_seq` back toward 0.5 (the disclosed
phased-curriculum second phase: more complete sequences for boundary
training); if a skill is flat at full exposure, its verdict is
"reward/start curriculum bottleneck", per the operator's decision
table — no capacity claim either way without seed twins + bulk
cohorts.

## Decision interpretation (operator's, binding)

- A passes, B fails → one-checkpoint composition solved;
  teacher/curriculum acquisition remains necessary.
- Both pass → larger/modular from-scratch hypothesis confirmed;
  seed twins + hardening.
- A fails before unfreezing → implementation/distill/transition
  defect, NOT PPO interference.
- A erodes only after unfreezing → keep frozen experts + adapter;
  do not force joint updates.
- B learns walk/lower but not rise despite full active exposure →
  rise reward/start curriculum is the bottleneck, not capacity.
- postlower3 (hw stance specialist) is NOT a substitute for either.
