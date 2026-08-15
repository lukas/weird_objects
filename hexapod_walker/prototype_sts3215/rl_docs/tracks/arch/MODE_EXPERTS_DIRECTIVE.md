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
