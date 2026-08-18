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

- Stage 0 (FINISHED 08-15 ~13:5x, train-1 CPUs; VERIFIED 08-15 ~15:2x
  — **FAIL, infrastructure not science, Stage 1 does NOT launch**):
  distilled rise/hold/lower experts from `footlow2_hard1` and loco
  from `bcgait1_hard1` — `distill_gru --experts --experts-adapter 32
  --transitions 300 --episodes 200 --dagger-rounds 2` (transdagger2's
  proven recipe, new teachers per the directive) →
  `ppo_goal_cw_arch_modeexperts_bc1.zip`. Training-time in-context
  teacher verification passed (12/12 det sequences, 0 falls) but the
  FULL VERIFY (single-mode det vs each teacher's bars + sequence eval
  vs c1) finds real misses: **det rise 0/6** (bridge 0/3, flat 0/3),
  stalling 40–80mm short of full stand (footlow2_hard1's own cold
  rises: 0.5–3.4mm) — a genuine miss, not noise, foreshadowed by the
  training log's own rise probe returns (`['-217','125']`, versus
  walk `['837','734']`/hold `['93','274']`). **det walk prog_ratio
  med 0.53** (teacher's own 1.05–1.10) with 2/6 episodes collapsing
  to prog 0.02/0.12 and slip/m 26.2/7.7 (teacher 1.3–1.5) — an
  intermittent near-total stall never seen in the teacher. hold/lower
  det clean (6/6 each, matching teacher bars). Sequence
  (`eval_modeseq --single`, grammar rise,walk,lower,rise,walk): det
  10/12 zero-fall (bar 11/12, just under — first-rise-ordinal 6/12
  vs later-rise 10/12, i.e. NOT the usual post-lower-weak pattern),
  sto collapses to 3/12. **Root-cause hint (not confirmed): the
  DAgger correction budget over-weighted lower's failures** (300-ep
  transitions pass: falls `{'lower':6,'rise':1,'walk':3,'hold':1}`)
  **while rise stayed under-corrected** — consistent with the
  cross-track insight already on file for the transdagger2/3 rise
  line (fb_20260814T164337_d7f11b: rise needs its own targeted
  BC-anchor/coverage term, not just a diet re-weight). Per the
  pre-registered FAIL branch below, this is infrastructure: **fix or
  re-collect the distill (rise-targeted DAgger coverage is the
  leading candidate), then re-run VERIFY — no Stage 1 PPO on this
  artifact.** Evidence: `logs/ckpt_eval/arch_modeexperts_bc1_verify`,
  `logs/ckpt_eval/arch_modeexperts_bc1_seq_{det,sto}.json`.
  **Stage 0 re-collection `bc2` LAUNCHED 08-15 ~17:0x UTC (idle-kick
  drain, no operator wait needed — this was the named `[code]`/
  `[precondition: distill recipe fix]` item):** `distill_gru` gained
  `--dagger-extra-mix`/`--dagger-extra-episodes` (default off, 4 new
  tests green + full gru_policy suite 25/25 green, snapshot
  `exp/arch-modeexperts-bc2-rise-dagger`) — a SECOND, single-mode
  targeted DAgger pass each round on top of the sequence one, so a
  mode that rarely triggers a hard fall (rise stalls short instead of
  falling, hence its near-absence from the bc1 fall tally) still gets
  extra correction density. Exact bc1 recipe
  (`--experts --experts-adapter 32 --transitions 300 --episodes 200
  --dagger-rounds 2`, same teachers) plus `--dagger-extra-mix
  rise=1.0 --dagger-extra-episodes 100`, seed 0 — ONE variable vs
  bc1. Running as a CPU job on train-0 idle cores (PID confirmed via
  /proc utime climbing; teacher-verify/collection prints are batched,
  same buffered-log profile as transdagger — do not mistake silence
  for a stall), `--out
  rl_move/sim/policies/ppo_goal_cw_arch_modeexperts_bc2.zip`, log
  `/tmp/modeexperts_bc2.log`. (Original plan, superseded by the
  RESULT below: re-run the same VERIFY before any Stage 1 PPO
  launch — if bc2 also misses the rise bar, escalate to `[operator]`
  rather than a third recipe variant.)
  **RESULT (08-16 ~11:5x UTC): VERIFY ran — MIXED, and per the
  pre-registered branch this is a SECOND MISS -> escalate to
  `[operator]`, no Stage 1 PPO, no third recipe variant.** The
  rise-targeted DAgger top-up DID move isolated single-mode rise the
  right way (det 0/6 -> **3/6**, real non-crouch wins this time:
  bridge 1/1, flat 1/1, rsi 1/3; hold 6/6, lower 6/6 both retained;
  walk gait honest, gv 6/6 det + 6/6 sto, prog 1.01/0.88) — but it is
  still short of the teacher bar (footlow2_hard1 cold rises stall
  0.5-3.4mm; this checkpoint's sto rise is still 0/6). **The sequence
  metric NET REGRESSED and changed character**: overall det
  zero-fall 10/12 (bc1) -> **6/12** (bc2); by ordinal, first-rise
  improved 6/12->7/12 but the POST-LOWER rise went from stalling
  short (bc1) to **actually falling 6/6 of its 6 failures** (rise_by_
  ordinal[1] = 6 success/6 falls out of 12) — sto is worse across the
  board (seq 3/12->4/12 barely, but first-rise sto is now 0/12, was
  some nonzero in bc1). Video (contact sheet,
  `logs/ckpt_eval/arch_modeexperts_bc2_verify_stance2/contact_sheet.png`)
  confirms genuine rise/hold/lower motion, no park/flag-leg exploit —
  this is a real skill trade-off, not a measurement artifact. **This
  is the SAME zero-sum DAgger-correction signature the transdagger3
  line already found** (topping up one segment's correction density
  measurably erodes another's) — now reproduced on the isolated
  4-expert architecture too, independent of the shared-GRU mechanism
  transdagger/dual2 blamed. Two misses (bc1: rise+sequence both miss;
  bc2: rise partially fixed, sequence gets WORSE via a harsher failure
  mode) closes the DAgger-recipe-variant ladder for Arm A Stage 0 per
  the pre-registered branch — no bc3. Open question for the operator:
  whether BC/DAgger distillation can produce a single checkpoint that
  holds isolated-rise AND post-lower-rise simultaneously at all under
  this architecture, or whether Stage 1 needs to start RL-based
  correction (unfreezing rise) rather than waiting for a better
  distill. Evidence: `logs/ckpt_eval/arch_modeexperts_bc2_verify_
  {stance2,walk}`, `logs/ckpt_eval/arch_modeexperts_bc2_seq_{det,sto}.json`.
  No Stage 1 PPO launched from this result.
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

**SCRATCH2 RESULT + SCRATCH3 EXECUTED (08-17 ~15:2x UTC, triage
cycle):** scratch2 finished its full 40.04M clean (no NaN/crash).
Exposure clause drifted exactly as pre-classified — at 10M rise .394
(over band), loco .283/lower .299 (in band), hold .024 (fine); at end
rise .312 (back in band), loco .236/lower .241 (just under band),
hold .211 (~2× the .15 cap) — driven by sequence episodes' hold
segment banking more ticks as rise got reliably completed (more
sequences now survive rise long enough to reach hold). Per-expert
stds diverged independently (hold 1.84 > rise 1.51 > lower 1.33 >
loco 0.74, shared 0.39 init) and per-mode in-loop eval trends
improved (hold survived_frac 0→~1.0 by ~12M; walk/lower stayed high
throughout). **Bulk harness fork read** (DR0 gate + DR0.5 own-cfg,
det+sto, 6 eps/mode): 0/6 success on all 4 skills both passes, but NO
proven dominating exploit — walk gait_valid 6/6 both passes (0
sacrificed legs), real forward travel matching the commanded speed
(1.4–1.8 m/30s) and prog_ratio ~1.0–1.1, just too much slip to clear
the bar (slip/m 1.8–2.4 vs a trained champion's ~1.0–1.3); rise curls
upward genuinely on video but over-currents in 3/6 det episodes both
passes (worst_clear ~100–140mm); lower descends genuinely but never
reaches the flat-plant target (worst_clear ~275–305mm, drag
~1.0–1.6m/ep, 0 terminations); hold stays upright and stable (roll
tail 0.4–0.8°) but off the target height band (worst_clear
~130–215mm). Reads as real, uniform under-training at ~9–13M active
ticks/skill so far — exactly the case this gate defers judging, not a
reward/eval bug.

Cumulative lineage active ticks (canary + scratch2):
rise 0.70+12.48=**13.18M**, loco 0.53+9.45=**9.98M**,
lower 0.44+9.65=**10.09M** — all still short of the ≥20M target
(need +6.8M / +10.0M / +9.9M respectively). Per-skill acquisition HAS
partially appeared (real, non-exploit motion in all 4 videos) but not
uniformly, and hold is already 2× over its cap purely from sequence
leakage even at `mode_seq=0.2` — raising `mode_seq` toward 0.5 now
would only inflate hold further and starve loco/lower more, so this
stage instead CUTS `mode_seq` 0.2→0.10 (less sequence-driven hold
leakage) and re-solves the single-mode mix from the measured
realized/commanded RATIO at end-of-run (realized_i/commanded_i:
loco 0.68×, lower 0.74×, rise 1.03×, hold 7.5×) inverted and
renormalized to target realized ≈.30/.30/.30/≤.10 →
**rise 0.303→0.254, walk 0.345→0.383, lower 0.324→0.352,
hold 0.028→0.011** (sums to 1.000). This is a pragmatic ratio-based
re-solve, not an exact stationary-f_seq solve (attempting the
original two-equation form on scratch2's non-stationary realized
values produced an impossible negative f_seq for loco — the
mix/hold-leakage relationship shifted over the run as rise got more
reliable, so a single constant f_seq cannot fit both the 10M and
end-of-run reads). At 40M new steps and a realized rate anywhere near
this estimate, rise/loco/lower each clear +10-12M new active ticks,
comfortably closing the ≥20M cumulative gap.

**`cw-arch-modeexperts-scratch3` LAUNCHED same cycle** (`respec --from
cw-arch-modeexperts-scratch2 --init-from-source --steps 40000000
--phase hardening --arg='--goal-mix=walk=0.383,rise=0.254,
lower=0.352,hold=0.011' --cfg goal.mode_seq=0.10 --now --pod
hexapod-mjx-train-2`), VERIFIED RUNNING (W&B `puvo5i2y`). Gate now
additionally requires a skill SUCCESS verdict at this fork (≥1/6 on
a skill that scored 0/6 in scratch2 = real progress; still 0/6 at
≥20M cumulative ticks = reward/curriculum bottleneck, not capacity).
Evidence: `rl_docs/runs/cw-arch-modeexperts-scratch2.md`, W&B
`1t6rmexz`, `logs/ckpt_eval/cw_arch_modeexperts_scratch2_{gate,owncfg}`.

**SCRATCH3 RESULT (08-18 ~19:2x UTC, triage cycle): STOP — known
exploit dominates, no scratch4.** Finished clean 40.04M, no NaN/crash.
Cumulative lineage active ticks reached the ≥20M target (rise
0.70+12.48+10.14=**23.32M**, loco 0.53+9.45+9.27=**19.25M**, lower
0.44+9.65+9.03=**19.12M**) — this is NOT the "still short of exposure"
case. Exposure clause (b) at end: rise .253/loco .231/lower .226 (all
~0.05-0.07 low of the .30 target — mild), **hold .290, a WORSE miss
than scratch2's .211** (nearly 2x the ~0.15 target this stage even
after cutting mode_seq 0.2→0.10 AND hold's single-mode share to
.011) — sequence-leakage into hold got worse, not better, at the
lower mode_seq. Per-expert std (clause c): loco converged healthily
to 1.13 (walk stayed genuinely good); **rise 4.73, lower 4.64, and
especially hold 13.48 (vs 1.84 last stage) — hold's std more than
7x'd**, a implementation/collapse-scale signal the gate's own clause
(c) flags.

**The bulk harness fork read is the real story: rise, hold, AND lower
have each independently collapsed into the IDENTICAL cheat** — plant
3 legs (duty_cycle ~0.85-1.0, end_clear ≈0mm), freeze the other 3 up
in the air for the ENTIRE episode (duty_cycle 0.0, end_clear
30-320mm), a static tripod hold. Confirmed on ALL 48 non-walk test
episodes (rise/hold/lower × det/sto × DR0/DR0.5) — every single
episode shows this exact duty-cycle signature, video-checked on
multiple episodes per mode per pass. hold is the most degenerate:
its deterministic action is byte-identical across all 6 nominally
different test episodes (`end_clear_mm=[317.3,113.0,-0.2,115.7,
-0.2,-0.1]` every time, leg 0 folded ~317mm off the ground) — the
policy ignores its input entirely. The 2/6 "success" flags logged on
`lower/det` under own-DR (0.5) are the SAME cheat at a smaller
raised-leg clearance (33-38mm vs 60-90mm elsewhere) that happened to
cross the coarse `end_posture_ok` threshold (this eval did not use
`--valid-plant-gate`) — video-confirmed identical posture pattern to
the "failed" episodes, not a real completion. Walk is unaffected:
gait_valid 6/6 both DR passes, genuine six-leg cycling, prog_ratio
1.0-1.1, matching scratch2.

**This is a REGRESSION vs scratch2**, whose read was genuine
(non-exploit) if incomplete motion on all three non-walk modes (rise
curled, lower descended, hold stood stable). Between scratch2 and
scratch3 the mix cut `mode_seq` 0.2→0.10 and hold's single-mode share
7.5x→.011 — apparently NOT enough to starve the sequence-leakage
pathway that feeds hold ticks, and the extra ~10-12M active ticks on
rise/lower let each converge into a reward-satisfying but
task-failing fixed point instead of continuing to close the gap.

Per `RUN_INTERPRETATION_RULES.md`, a known exploit dominating video is
a complete verdict at any behavioral checkpoint, and the SCRATCH3
gate's own pre-registered kill clause ("a proven exploit dominating a
milestone video") fires here: **no scratch4, no re-run with more
steps.** Root-cause hint for whoever reopens this line: `walk`'s
reward carries explicit anti-park/anti-flag-leg terms
(`reward.k_step_event`, `k_drag_loaded`, `k_park_duty`,
`walk_kernel_prog_gate`, `walk_anchor_gate`) that `rise`/`hold`/
`lower` apparently lack an equivalent of — nothing in those three
modes' reward currently prices "sacrifice half the legs and freeze,"
so once an expert finds the fixed point under isolated-expert
training (where it can't be pulled back out by shared-parameter
interference from the other experts, unlike the shared-trunk
lineages) it has no incentive to leave. Banking a tripod/flag-leg
ordering clause into rise/hold/lower's `test_task_semantics.py` (the
MDP_PREFLIGHT bank), mirroring walk's, is the concrete next lever —
not queued this cycle (SIM SPRINT: no new arch launches unless
sim-rise/walk-serving). Evidence: `rl_docs/runs/cw-arch-modeexperts-
scratch3.md`, W&B `puvo5i2y`,
`logs/ckpt_eval/cw_arch_modeexperts_scratch3_{gate,owncfg}` (contact
sheets + per-episode `duty_cycle`/`end_clear_mm` are the smoking gun).

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
