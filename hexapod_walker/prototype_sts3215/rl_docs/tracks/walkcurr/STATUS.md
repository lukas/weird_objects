# walkcurr — prior-free walking curriculum (Kawawa-2022 lineage)

Registered 2026-08-23 by operator order (MCP focus note
20260823T154657Z) after the `cw-kawawa2022-pf-flat1` FAIL. Plain
English: teach a from-scratch PPO policy (no gait clock, no BC
teacher, no motion prior) to walk by climbing a curriculum that starts
with ONE fixed forward command and only widens after certified passes.

## Goal (DONE gate)

A prior-free policy passes a held-out C-env contextual walking panel
(fixed forward + heading set + irregular direction changes) with zero
falls, directions actually followed, low slip/m, all-six-leg gait
validity, on video. Speed obedience is secondary throughout.

## Binding track rules (operator, 08-23)

- **Walk-only diet**: every rung trains with `goal.walk_pure=1`. The
  flat1 failure mode (hold/raise/track/unload carrying aggregate
  reward while walk dies) must be impossible by construction.
- **Bank before launch**: any reward-mechanism change re-proves the
  WALKCURR_PF ranking bank (test_task_semantics.py): clean commanded
  walking > park/stall > sideways/reverse/wrong-way >
  high-slip/skate/fall, under the run's exact cfg.
- **Triage rule**: every triage logs reward trend AND walk-eval trend.
  Reward rising while walk eval is flat/down or walk terminates =
  MISALIGNED -> stop same-recipe seeds/continuations, audit
  reward/eval/simulator. No same-recipe seed sweeps past a misaligned
  read.
- Slip is priced by charge (loadslip excess), never by a hard early
  gate that teaches parking, unless bank and eval agree.

## Rung ladder

1. **fwd1 (NOW)**: fixed forward 0.05-0.06 m/s, heading 0, DR0,
   discovery 2M. Recipe: `rl_move/sim/kawawa2022_recipe.py`.
2. Small heading set (± up to ~15 deg), one command/episode.
3. Full fixed headings.
4. Irregular direction changes (mid-episode resampling).
5. DR/push hardening (paper's friction 0.5-1.25 + periodic pushes).

## Now

- **`cw-walkcurr-pf-fwd3-chargeramp` FAIL (verdicted 08-23 ~18:3x):**
  the dense-charge ramp fix did NOT unfreeze discovery either — gate
  eval 0/6 walk success (det+sto), prog_ratio ~0.00, speed
  0.003-0.034 m/s vs 0.05-0.06 cmd, slip/m 10.4-39.2 (gate <=3.0),
  contact sheet static splayed stance across all 10 frames — same
  pathology as fwd1/fwd2. `env/walk_freeprog_score` (see metric note
  below) sat flat NEGATIVE (~-0.06..-0.10) the entire 2M steps, never
  crossing zero. This is the THIRD independent reward-magnitude/
  schedule fix (fwd2 swing income, fwd2 term-penalty cut, fwd3 charge
  softening) refuted with the same frozen-video signature — the
  blocker looks upstream of reward shape: random-init PPO rollouts
  never visit a state with positive forward progress to reinforce, so
  no reshaping of charges the policy never triggers can matter.
  **METRIC DEFECT found (fwd3 triage):** the discovery-health litmus
  this section's gate text has been citing, `env/reward_walk_prog`,
  is a DEAD METRIC under this recipe — `walk_task.py` sets
  `r_prog = 0.0` unconditionally whenever `k_walk_freeprog > 0`
  (freeprog REPLACES it), so it reads exactly 0.0 regardless of
  actual behavior on every fwd1/fwd2/fwd3 run. It happened to agree
  with the true (video-confirmed) verdict every time so far, but is
  not a valid go/no-go signal — **use `env/walk_freeprog_score`
  (or `walk_freeprog_score` in eval reports) instead in all future
  rung-1+ gate text.**
  **UNTRIED lever surfaced:** `k_loadslip_excess` is EXCLUDED from
  `walk_charge_ramp` entirely (always full dose — a pre-launch safety
  fix after floor 0.15 inverted the skate/shuffle ranking at
  convergence-level pricing). It is the exact charge fwd1's dig-in
  named (the loadslip "travel-floor" that prices a tiny exploratory
  step's slip/progress ratio harshly since progress is floored at
  `loadslip_floor_m`=0.03 m) and no arm has yet tried softening it,
  even as a short bootstrap window (distinct from a permanent low
  floor, which the bank already rejected at convergence).
  Snapshot `exp/walkcurr-fwd3-chargeramp` (unchanged, FAIL result
  only).
  **`cw-walkcurr-pf-fwd4-logstd0` / `-fwd4-entboost` LAUNCHING
  (08-23 ~18:3x):** the pre-registered init/exploration fork, 2-arm
  dose pair off the exact fwd3 recipe (charge ramp kept — it's
  harmless and bank-proven safe, just not sufficient alone):
  (a) `--log-std-init 0.0` (std 0.37->1.0, ~2.7x wider initial action
  noise) to test whether the random-init rollout distribution simply
  never samples a forward-progress trajectory at the current noise
  scale; (b) `--ent-coef 0.01` (10x default 1e-3) to test whether
  entropy collapses to the frozen/park optimum before progress-
  bearing states are ever visited. Single-lever each, no reward-
  mechanism change (no bank re-proof required). Prediction-if-true:
  `walk_freeprog_score` leaves its flat ~-0.07 band and trends toward
  0 within 2M. Prediction-if-false (both freeze identically): the
  loadslip-bootstrap lever above becomes the next fork to build, or
  the prior-free MLP recipe is refuted at this budget per the rule
  below.

## Now (updated 08-23 ~20:0x)

- **`cw-walkcurr-pf-fwd4-logstd0` / `-fwd4-entboost` FAIL (both
  verdicted):** wider initial action noise (std 0.37->1.0) and 10x
  entropy coefficient (0.001->0.01) both leave `walk_freeprog_score`
  flat in [-0.10,-0.055] the whole 2M run — never trending toward 0.
  Gate eval 0/6 det on both, identical static splayed-crouch video to
  fwd1-fwd3. `fwd4-logstd0` additionally destabilized sto-mode into
  real falls (5/6 term, tilt_roll/tilt_pitch, one sacrificed leg) — a
  new, worse failure mode layered on the same freeze, not progress.
  Exploration-noise-scale hypothesis CLOSED.
- **`cw-walkcurr-pf-fwd5-loadslipboot300k` / `-600k` FAIL (both
  verdicted):** built + bank-proved + smoke-tested the pre-registered
  loadslip-bootstrap mechanism (`reward.walk_loadslip_bootstrap_steps`
  / `_min_frac`, walk_task.py + train_ppo_mjx.py, 6 unit tests +
  3 semantics-bank tests, smoke-verified end-to-end on-pod) — softens
  ONLY `k_loadslip_excess` for an early window, annealing back to full
  dose. **Bank floor-search finding**: 0.40 (the walk-charge-ramp's
  own validated floor) is UNSAFE for this charge alone — held statically
  at 0.40 with the other three discovery-friction charges at full dose,
  'sideways' (-231) out-earns 'park' (-352), a genuine ranking reversal
  (unlike the ramp's own 0.15/0.40 story, because nothing else is
  loosened alongside it here). A 0.5-0.8 sweep found **0.65** the
  lowest safe floor (park/stall stay strictly above every wrong-way
  gait, margin ~37; skate charge still cut ~30%, -1339->-940 at full
  dose vs floor). Trained at 0.65 for both 300k (15% of budget) and
  600k (30%) windows: **both freeze identically** —
  `walk_freeprog_score` flat in [-0.10,-0.05] the entire run, gate 0/6
  det+sto, identical static-crouch video. Window length was not the
  missing variable. **This closes the loadslip-bootstrap fork at both
  tested doses.**
- **Escalation triggered**: per the pre-registered rule below, BOTH
  init/noise (fwd4, 2/2 FAIL) AND loadslip-bootstrap (fwd5, 2/2 FAIL)
  are now refuted with aligned reward+eval (reward falls monotonically
  as the charges ramp against a frozen policy; eval flat/0 the whole
  way; adequate budget — 2M steps, 6 total rung-1 arms: fwd1, fwd2x2,
  fwd3, fwd4x2, fwd5x2 is 8 arms total, all FAIL). **The prior-free
  MLP-from-scratch recipe is refuted at the 2M discovery budget.**
  Escalate to a dig-in before any further reward-magnitude variant —
  see Next.

## Next

- **`cw-walkcurr-pf-fwd6-rscale50-cont1` FAIL (verdicted 08-23 ~21:3x)
  — the optimizer-crush fix is necessary but NOT sufficient; the
  pre-registered RUNG-0 escalation FIRES and THIS cycle owns it.**
  +4M continuation (6M total at x0.02): `walk_freeprog_score` never
  left [-0.10,-0.046] (final -0.075, zero crossing never reached),
  det gate 0/6 (prog med -0.01, fwd 0.00 m, gait_valid 0/6, 0 terms),
  det strip = the identical static splayed crouch of all 9 frozen
  rung-1 arms — while clip_fraction stayed healthy (0.07-0.11) and
  value_loss O(1). Gradients flow; PPO still prefers stillness. This
  also resolves the parent's "strongest alternative": the earlier
  freeprog rise was stillness optimization saturating below zero.
  Combined with the swing-bank part-2 finding (income-side levers
  exhausted at bank-legal doses inside the rung-1 diet), the blocker
  is the rung-1 reward LANDSCAPE around park, not optimization.
  **Escalation choice (recorded, assume-and-go): rung-0 sub-goal
  before RND** — change the diet so leg-cycling IS the certified
  goal: swing income (`k_walk_swing`, any-direction completed swings)
  as the dominant term, travel-demanding charges (idle/heading)
  REMOVED, freeprog at 10% dose (keeps the legacy speed kernel
  replaced + a slight forward preference), park_duty/loadslip/term
  kept, everything at the crush-proven x0.02 scale. Certification
  gate: zero falls + six legs cycling (gait_valid) on the det panel +
  video shows rhythmic stepping; travel NOT required. Then rung-1b
  warm-starts the rung-1 diet from the certified stepper. RND
  state-novelty stays the fallback if rung-0 also freezes.
  **Coordination: the cycle triaging `rscale10-cont1b` should read
  ONLY the x0.1 dose question (freeprog zero-crossing per its own
  gate) — the escalation is owned here; no second escalation build.**
  Rung-0 bank (stall/shuffle/gait > park/stork; skate/topple floor;
  stall lifetime-positive) measured + landing in
  `test_task_semantics.py` this cycle before launch.
- **DIG-IN COMPLETE (08-23 ~20:4x, deep cycle): the freeze signature
  is CONFIRMED 8/8 and root-caused one level deeper — to the
  OPTIMIZER, not the reward shape or exploration.** (1) All 8 FAILed
  rung-1 arms show the identical collapse in their cached W&B
  histories: `train/clip_fraction` reaches EXACTLY 0 by 10-38% of the
  run and never recovers (fwd4-logstd0 — the WIDEST init noise —
  collapsed fastest, at 10%), approx_kl ~3e-5, final policy std at
  its init value (0.3685 ~ e^-1). (2) Mechanism: `train/value_loss`
  sits at 400->2000+ because v2e returns are |1000s|-scale; SB3 PPO
  clips the GLOBAL grad norm (max_grad_norm=0.5) over policy+value
  parameters in ONE optimizer, so a value head chasing thousands-scale
  returns dominates the norm and rescales the policy/log_std/entropy
  gradients toward zero. SB3's per-minibatch advantage normalization
  makes the policy gradient scale-invariant, so ONLY a global reward
  down-scale (or a grad-norm/vf_coef change) relieves the crush —
  and it explains why pricing, init-noise, and entropy levers all
  failed IDENTICALLY: they act downstream of it (10x ent_coef moved
  std just 0.369->0.376 in 2M). Even at update 1 clip_fraction was
  only ~0.013 (healthy PPO: 0.05-0.2). (3) **Bonus reward-stack
  finding (scaled-bank measurement)**: the walk diet is NOT gain-pure
  — `reward_task` (base compute_reward posture kernel, k_track=1.0
  default, NOT in the recipe cfg) pays ~+0.95/step to EVERY
  level-bodied behavior including park and skate (+348..+366 per 15 s
  probe). Scaling only the 7 recipe gains would have DESTROYED the
  ranking (park +323 at x0.1); the full scale set (7 recipe gains + 8
  base compute_reward gains) is bank-GREEN at x0.1 and x0.02 with
  margins scaled and behavior deltas linear
  (`test_walkcurr_pf_scaled_*`, landed this cycle).
- **LAUNCHED (this cycle): `cw-walkcurr-pf-fwd6-rscale10` /
  `-fwd6-rscale50`** — global reward scale x0.1 / x0.02 on the exact
  fwd3-chargeramp recipe (all 15 active gains scaled together;
  ranking mathematically and bank-measurably unchanged; no bootstrap,
  single lever). Prediction-if-true: clip_fraction stays healthy
  (>~0.02) past 50% of the run, std moves off init, and
  walk_freeprog_score (gain-independent metric) leaves the
  [-0.10,-0.05] band. Prediction-if-false (freeze recurs WITH healthy
  clip_fraction): the optimizer-crush theory is refuted and the
  exploration/curriculum escalation below is next.
- **Concurrent fwd6 batch (other cycle, 08-23 ~20:3x):**
  `fwd6-sde` (gSDE state-dependent exploration, finished — awaiting
  triage), `fwd6-gru` (recurrent, training), `fwd6-budget5m`
  (REFUSED on a dirty code marker; relaunch if still wanted). CROSS-
  PREDICTION registered by this dig-in: if the optimizer-crush theory
  is right, sde/gru/budget arms ALL freeze again (their gradients are
  crushed by the same global clip regardless of exploration structure,
  memory, or budget) while the rscale arms unfreeze — the two theories
  (action-insensitive-reward advantage starvation vs value-gradient
  crush) are now cleanly discriminated by one wave.
- **`cw-walkcurr-pf-fwd6-rscale10-cont1b` FAIL (verdicted 08-23
  ~21:27): crush-fix necessary-but-INSUFFICIENT at the x0.1 dose.**
  The +4M continuation kept the optimizer healthy the whole way
  (clip_fraction 0.03-0.16, value_loss ~6, std moving off init) —
  the first rung-1 lineage without the collapse — yet
  `walk_freeprog_score` never left the [-0.05,-0.08] band over 6M
  total, det gate 0/6 at 0.002 m/s, identical static-crouch video
  (leg 2 sacrificed). x0.1 same-recipe lineage CLOSED per its own
  pre-registered branch. The escalation fork below now hinges
  SOLELY on `rscale50-cont1` (x0.02, finished, its own cycle's
  triage): parent's freeprog rose monotonically -0.103->-0.015, so
  a zero-crossing there keeps the scale lever alive (dose-
  sensitive); a sub-zero plateau there fires (a)/(b) below.
- **If the scale arms fail**: escalate in this order — (a) prior-free
  exploration fix (curiosity/RND state-novelty bonus, or gSDE — a
  concurrent cycle landed `--use-sde` validation support 08-23; or a
  5-10M discovery budget); (b) rung-0 sub-goal ("lift any foot" /
  "shift weight" certified before full walking); (c) `--gru` recurrent
  path; (d) last resort, operator-flagged: brief BC kickstart (brushes
  the track's "no BC teacher" rule).
- **PARALLEL FORK LAUNCHED, same cycle as the rscale pair (before this
  cycle read the optimizer-crush write-up above — both read the same
  clip_fraction-collapse symptom independently and reached for
  different items on the (a)/(c) list): `cw-walkcurr-pf-fwd6-sde`**
  (single lever `--use-sde`, gSDE temporally-correlated action noise
  instead of i.i.d. per-tick noise, on the exact fwd3-chargeramp
  recipe) **and `cw-walkcurr-pf-fwd6-gru`** (single lever `--gru
  --n-steps=64`, recurrent actor/critic, same recipe otherwise) — both
  VERIFIED RUNNING (train-1, train-2), 2M each, same rung-1 gate.
  Built + unit-tested `--use-sde`/`--sde-sample-freq` in
  `train_ppo_mjx.py` (4 construction sites threaded, default OFF
  bit-exact, mirrors `--activation-fn`'s from-scratch-only guard;
  `test_use_sde_flag.py` 4/4 — includes a `--help` subprocess
  regression test that caught a real bug while writing the help text:
  a literal `~15%` in a docstring crashes argparse's formatter at
  `--help` time, `TypeError: %o format`). **HONEST CAVEAT given the
  optimizer-crush write-up above**: neither gSDE (noise STRUCTURE)
  nor GRU (memory) changes the mechanism the crush write-up blames
  (global grad-norm clipping dominated by a large-scale value loss) —
  if that theory is right, both of these arms predict-if-false FAIL
  for the SAME underlying reason as fwd1-5, not a fresh refutation of
  noise-structure/memory specifically. A PASS on either would be the
  more informative (and surprising) outcome; a FAIL should be read
  jointly with the rscale arms' verdict before concluding
  noise-structure/memory are closed classes — if rscale FIXES the
  freeze, re-test gSDE/GRU restricted to whether they change anything
  ADDITIONAL on top of the rescaled reward before fully closing them.
- Once ANY rung-1 mechanism actually gates: rung 2 (small heading set)
  respec; consider the recurrent path + paper-pure proprioception-only
  obs A/B (`goal.walk_obs_body_vel=0.0`) once commands start changing
  mid-episode (rung 4).

## Now (updated 08-23 ~20:5x)

- **`cw-walkcurr-pf-fwd6-sde` FAIL (verdicted):** gSDE (temporally-
  correlated per-rollout action noise instead of i.i.d. per-tick) does
  NOT unfreeze rung-1 — identical static splayed-crouch to fwd1-fwd5.
  Gate eval 0/6 det success, det speed 0.003 m/s vs 0.05-0.06 cmd,
  dir_err 52.8deg, slip/m 9.46, prog_ratio 0.00 (sto 0/6 too). W&B
  history: `train/clip_fraction` collapses 0.035->0 by ~50% of the
  2M run and stays there; `train/std` never leaves its log-std-init
  value (0.3673 the whole run); `env/walk_freeprog_score` flat in
  [-0.18,-0.15] the entire run (flatter/worse than the [-0.10,-0.05]
  prior band, never trends toward 0). This is exactly the cross-
  prediction the optimizer-crush dig-in pre-registered: gSDE changes
  noise STRUCTURE only, and does nothing about the mechanism blamed
  (SB3's single global grad-norm clip dominated by |1000s|-scale
  value loss crushing policy/log_std gradients regardless of noise
  shape). **Noise-structure hypothesis CLOSED alongside noise-
  magnitude/entropy (fwd4) and loadslip-bootstrap (fwd5) — 9/9
  rung-1 arms now FAIL with the identical clip_fraction-collapse
  signature.** `fwd6-gru` (recurrent, another concurrent-cycle arm)
  is the last untriaged member of this batch; per the same cross-
  prediction it should fail identically unless memory specifically
  breaks the crush. The `fwd6-rscale10`/`-rscale50` siblings (reward
  down-scale — the dig-in's actual proposed fix) remain the live
  discriminating test; do not fund further noise-structure/
  exploration variants (e.g. gSDE sample-freq sweeps) until they
  read. If rscale ALSO fails with healthy clip_fraction, escalate to
  rung-0 sub-goal or RND per the pre-registered order above.

## Now (updated 08-23 ~21:1x — fwd6 wave read complete except rscale50)

- **OPS NOTE**: `fwd6-sde` was double-verdicted (this cycle + a
  concurrent one, same FAIL conclusion, minutes apart — 3rd
  double-triage today). Verdicts agree; no conflict to resolve.
- **`cw-walkcurr-pf-fwd6-gru` FAIL (verdicted):** recurrence
  refuted — GRU RecurrentPPO freezes identically (gate 0/6 det,
  prog 0.00, static splayed crouch; clip_fraction 0.027@512k ->
  0.0007@768k -> exactly 0 for the rest; std pinned 0.369;
  freeprog flat [-0.09,-0.06]; value_loss 880-2400). The GRU value
  head faces the same |1000s|-scale returns and the same global
  max_grad_norm clip — memory is exonerated, crush cross-prediction
  holds.
- **`cw-walkcurr-pf-fwd6-rscale10` PARTIAL (verdicted): the
  optimizer-crush theory is CONFIRMED mechanically at x0.1.** Only
  arm of the wave whose clip_fraction never hit exactly 0 and
  RECOVERED (0.021@1632k -> 0.079@2016k, rising; value_loss 8-18 vs
  400-2400 unscaled; approx_kl 0.02) — but recovery came only in the
  final ~15% of 2M, and behavior is still the frozen crouch (gate
  0/6 det, det pose holds leg 3 up -> sac=[3]; freeprog left the
  -0.10 start but hovers -0.05..-0.07). ~300k effective
  post-recovery steps is too few to expect discovery.
- **[RESOLVED while this section was being written: cont1b finished
  its +4M in minutes and was verdicted FAIL by a concurrent cycle —
  see the newer entry above. Optimizer healthy the whole run, policy
  still frozen: crush necessary-but-insufficient at x0.1; escalation
  decision rides on `rscale50-cont1` (x0.02, running).]**
- CONTINUATION (launched this cycle): `cw-walkcurr-pf-fwd6-rscale10-cont1b`
  (+4M acquisition, byte-identical cfg, `--init-from` the rscale10
  checkpoint; first launch attempt `-cont1` FAILED W&B verification,
  self-repair relaunched as `-cont1b` on train-0). Pre-registered
  fork: freeprog leaves [-0.10,-0.05] toward 0 = discovery started;
  flat ANOTHER 4M with clip_fraction healthy = optimizer-crush
  necessary-but-insufficient -> escalate to RND state-novelty /
  rung-0 sub-goal, same-recipe lineage closed. Note the 1M charge
  ramp restarts with the continuation (bank-proven pricing; softer
  charges during renewed exploration only help discovery).
- **`fwd6-rscale50` (x0.02) finished ~20:54, unverdicted — next
  cycle's read**: gives the dose comparison (does a stronger
  down-scale un-crush from step 0 instead of the last 15%?). Do NOT
  fund further exploration/architecture variants before rscale50 +
  cont1b read; do NOT relaunch `fwd6-budget5m` (unscaled budget arm
  is moot — the crush is dose-, not budget-, limited).

## Now (updated 08-23 ~21:1x)

- **`cw-walkcurr-pf-fwd6-rscale50` PARTIAL (verdicted): the
  optimizer-crush fix WORKS mechanically at x0.02, and this is the
  first rung-1 arm ever with a rising discovery signal.** Cached W&B:
  clip_fraction healthy every quarter (means 0.024/0.043/0.050/0.049,
  last 0.083 — never the exactly-0 collapse of the 9 frozen arms),
  std creeping off init (0.3678->0.3729), `walk_freeprog_score`
  monotonic -0.103 -> -0.015 by quarter — left the [-0.10,-0.05] dead
  band and trending toward the zero crossing. Gate eval still 0/6 det
  (fwd 0.01 m, static splayed crouch on video), so behavior hasn't
  crossed into stepping at 2M. **Dose read**: rscale50 (x0.02)
  strictly dominates sibling rscale10 (x0.1: freeprog flat -0.055,
  Q2 clip near-collapse 0.003) — the "x0.02 overshoots into the
  value-net noise floor" alternative is REFUTED; the stronger scale
  is the one trending.
- **`cw-walkcurr-pf-fwd6-rscale50-cont1` LAUNCHED (+4M warm-start,
  acquisition phase, train-1, VERIFIED RUNNING)** — byte-identical
  cfg per the parent gate's own 08-21 continue branch. Startup
  verified healthy at 1.5M: clip_fraction 0.093, std 0.385 (already
  past parent final), value_loss 0.625 (crush fully gone). Decision
  gate: freeprog crosses 0 and keeps rising = discovery live;
  plateaus <0 over the final 2M with clip >0.02 = crush-fix
  necessary-but-insufficient -> pre-registered RND/rung-0 escalation,
  no further same-recipe continuations.
- **RESPEC GOTCHA (cost sibling cont1 a dead launch)**: respec of any
  fwd3-lineage run into an `--init-from` continuation MUST pass
  `--arg='--activation-fn='` (empty is a valid argparse choice) —
  the trainer fatally refuses `--activation-fn elu` combined with a
  plain `--init-from` (checkpoint keeps its own activation). Also:
  discovery caps at 2M steps; continuations go `--phase acquisition`
  with `--evidence` naming the healthy canary + precedent.
- Do NOT fund further noise-structure/exploration/dose variants while
  the two continuations (rscale10-cont1 — other cycle; rscale50-cont1)
  are in flight; they are the discriminating test.

## Now (updated 08-23 ~22:4x — rung-0 swing income CLOSED, RND wave)

- **`cw-walkcurr-pf-rung0-swing3` / `-swing9` BOTH FAIL (verdicted):**
  the rung-0 "lift your feet and step, anywhere" sub-goal, at both a
  1x and 3x `k_walk_swing` dose, does NOT certify — but each dose
  fails a DIFFERENT static-pose way: swing3 converges to a one-leg-
  planted "stork" lean (5/6 legs sacrificed, one leg holds 0.96 stance
  duty forever, cheap swing-event income farmed without ever bearing
  load on the lifted legs); swing9 converges to an all-six-legs-
  airborne static hover (every leg near-zero duty, avoiding
  park_duty/loadslip charges entirely by never touching down). Both
  reads are ALIGNED per the 08-21 ruling (reward AND eval decline
  together, optimizer stays healthy — clip_fraction never collapses,
  std creeps up) — genuine refutations, not undertraining. **Swing-
  income-as-rung-0-mechanism is CLOSED at both bank-legal doses.**
  RND state-novelty (the pre-registered fallback) is next.
- **RND (Random Network Distillation) BUILT + LANDED**:
  `rl_move/sim/rnd_vec.py` (`RNDVecWrapper`, standard Burda et al.
  predictor/target MLP pair + running-std intrinsic blend, trained
  online each rollout) + `--rnd-coef`/`--rnd-hidden`/`--rnd-out-dim`/
  `--rnd-lr`/`--rnd-buffer`/`--rnd-train-steps`/`--rnd-train-batch`
  wired into `train_ppo_mjx.py` (default 0.0 = bit-exact OFF, no
  wrapper/predictor/target constructed) — `test_rnd_vec.py` 8/8 green
  (off-path rejection, blend math, novelty-decay claim itself, ring
  buffer, running stats, save/load round-trip). Treated as a pure
  exploration mechanism (like `--use-sde`/`--gru`) that touches no
  `k_*` walk charge — no WALKCURR_PF ranking-bank re-proof required
  (the bank prices charge orderings; RND adds an orthogonal intrinsic
  term with its own dedicated mechanism tests instead).
- **RND WAVE (2 concurrent cycles, partially overlapping designs —
  logged honestly): `cw-walkcurr-pf-fwd6-rnd02` FAIL (verdicted by a
  concurrent cycle) — RND directly on the rung-1 rscale50 diet (no
  rung-0 detour) at `--rnd-coef=0.02` does NOT unfreeze walking, and
  finds a THIRD independent route to the SAME all-legs-airborne-hover
  cheat swing9 found on rung-0: 0/6 gait_valid, forward_dist ~0.013m,
  mechanism itself works exactly as designed (rnd/intrinsic_mean
  decays 0.037->0.0055 as the predictor catches up; clip_fraction
  healthy 0.03-0.06) but the dose is insufficient to disturb the
  crouch/hover local optimum. **Cross-mechanism finding: airborne
  hover is now confirmed on THREE independent recipes (rung-0 swing9,
  rung-1 RND) — it looks like a structural escape hatch in this
  reward family (leaving the ground voids every ground-contact-linked
  charge: loadslip, park_duty, drag) rather than an artifact of any
  one mechanism.** `cw-walkcurr-pf-fwd6-rnd10` (0.10 dose, 5x) is
  FINISHED, unverdicted (same concurrent cycle's arm).
  **This cycle's own arms (launched before reading the sibling
  cycle's `rnd02`/`rnd10` — partial dose overlap, noted for the
  record rather than hidden):**
  `cw-walkcurr-pf-rung0-swing3-rnd1` (rnd-coef 0.02 on the rung-0
  swing3 diet) / `-swing3-rnd3` (0.06) — both FINISHED, unverdicted
  (next cycle's read; tests the RND-fixes-the-EASIER-in-place-cheat
  question directly, distinct from the rung-1-direct arms above).
  `cw-walkcurr-pf-fwd6-rscale50-rnd1` (0.02 — duplicates `rnd02`'s
  design almost exactly, expect the same airborne-hover FAIL; kept
  for independent-seed confirmation rather than killed once
  discovered mid-flight) / `-rscale50-rnd3` (0.06 — fills the real
  gap between the sibling cycle's 0.02/0.10 dose points) — both
  RUNNING. **Decision on close: if the airborne-hover cheat recurs at
  EVERY RND dose tested (0.02/0.06/0.10) on EITHER base, RND is
  refuted as a rung-1/rung-0 unfreezer at any bank-safe-adjacent dose,
  and the next fallback is a direct ground-contact/no-fly charge
  (price near-zero total foot contact directly, closing the hover
  escape hatch structurally) before reaching for item (d)'s brief BC
  kickstart.**

## Now (updated 08-23 ~22:4x, addendum — hover mechanism refined)

- **`cw-walkcurr-pf-fwd6-rnd02` FAIL (verdicted, this cycle):**
  confirms the sibling cycle's prediction exactly — RND at 0.02 on
  the rung-1 rscale50 diet reproduces the identical airborne-hover
  cheat (own-cfg C-env det panel 0/6 gait_valid, every leg duty
  0.02-0.04, fwd_dist ~0.013 m/25 s; sto also 0/6). Mechanism
  telemetry confirms RND itself works as designed (rnd/intrinsic_mean
  decayed cleanly 0.037->0.0055 as the predictor caught up,
  clip_fraction stayed healthy) — the dose is just too small to move
  this local optimum.
- **MECHANISM REFINED (eval-only read, no training cost): it is a
  BELLY-SIT COLLAPSE, not a true mid-air hover.** `height_err_end_mm`
  on the hover episodes is 116 mm (a large, sustained height error)
  while `roll_peak_deg` is only ~2.2 deg and `roll_tail_deg` is 0 —
  rock-stable, not falling. Cross-checked against `walk_task.py`:
  there is a tilt (roll/pitch) termination but **no height-based
  termination at all** — a policy that lets the body settle low
  (resting weight off the legs, splayed for lateral stability) never
  triggers a safety stop and only pays `k_height`'s (dose 2.0) linear/
  quadratic charge, which is evidently cheap relative to what it
  saves by avoiding essentially all leg-load-linked charges
  (`k_park_duty`, loadslip stack, drag) at once. This sharpens the
  "direct ground-contact charge" fallback named above into two
  concretely testable, cheap (eval-only first) next steps for
  whichever cycle picks this up once the RND grid is fully read: (1)
  bisect whether raising `k_height` alone (bank-legal dose, single
  lever) prices the settle out without new code; (2) if not, that is
  the evidence for a genuinely new mechanism — either a height-based
  termination (mirrors the existing roll/pitch safety pattern) or a
  direct minimum-total-foot-contact charge — either needs a new
  scripted "sit"/"belly-collapse" behavior added to the
  `test_task_semantics.py` harness (`_slipwalk_rollout` has no such
  policy today; "stork"/"topple" are the closest existing scripts but
  neither matches this pose) before it can be bank-proven.

## Now (updated 08-23 ~22:5x — rnd02/rnd10 double-triaged, dose bracket extended)

- **OPS NOTE**: `cw-walkcurr-pf-fwd6-rnd02`/`-rnd10` were double-verdicted
  (a concurrent cycle recorded FAIL at 22:39/22:40 with the sharper
  "airborne-hover/belly-sit collapse" diagnosis already in this file's
  22:4x sections; this cycle independently recorded FAIL at 22:53/22:54
  before spotting the prior entries, using plainer "static crouch"
  language for the same pose — same conclusion, no conflict, same
  precedent as today's earlier `fwd6-sde` double-triage). Cross-check:
  both readings agree on every number that matters — gate 0/6 det+sto,
  all 6 legs sacrificed, clip_fraction healthy throughout (no crush),
  rnd/intrinsic_mean decaying cleanly (mechanism works, dose doesn't).
  This cycle's read adds one quantitative point not yet in the file:
  `env/walk_speed` **decays** from an initial-noise ~0.10 m/s down to
  0.006-0.0058 m/s over the full 2M run on BOTH doses (0.02 and 0.10
  are within noise of each other, final speeds 0.00647/0.00578), and
  `direction_err_deg` sits flat at chance level (~88-92deg) the whole
  time — the policy is converging AWAY from locomotion, not
  approaching it despite freeprog trending toward 0; ep_rew_mean
  quarters for rnd02 actively decline (44.0/42.2/32.9/18.2). This
  reinforces (does not revise) the existing belly-sit/airborne-hover
  verdict: freeprog's approach to 0 is the reward of settling into the
  charge-avoiding static pose, not nascent walking.
- **LAUNCHED (this cycle, before spotting the double-verdict —
  still non-redundant with the pending 0.02/0.06/0.10 dose grid on
  the other base): `cw-walkcurr-pf-fwd6-rnd100`** (fresh, `--rnd-coef
  1.0`, 10x beyond rnd10 / 50x beyond rnd02 — makes the intrinsic term
  the dominant per-step reward rather than a minor addition; tests the
  overshoot-into-flailing alternative at last, since 0.02/0.06/0.10
  found zero qualitative difference) **and `cw-walkcurr-pf-fwd6-rnd10-cont1`**
  (+4M acquisition warm-start from `rnd10`'s own checkpoint, same
  `--rnd-coef 0.10`, byte-identical cfg — tests the budget axis: does
  the still-rising-but-not-crossed freeprog trajectory (-0.101→-0.0054)
  eventually tip over, or was it asymptoting below the belly-sit
  optimum). Both VERIFIED RUNNING (train-1, train-2 — a stuck INTENT
  on the cont1 launch, caused by this cycle's own `--now` command
  hitting its wrapper's 120s timeout mid-verification after the
  trainer was already alive on-pod, was repaired by hand: `checkup`
  confirmed HEALTHY, wandb id read directly from the pod's `wandb/`
  dir via `kubectl exec`, ledger patched via `launch_run.py update
  --set status=RUNNING --set wandb_id=... --set verified=...`).
  **Decision context**: per this file's own 22:4x conclusion, if
  BOTH of these also reproduce the airborne-hover/belly-sit pose (not
  just "still frozen"), RND-as-a-class is refuted across the full
  dose range (0.02→1.0) AND the budget axis, and the next fallback is
  the already-named **direct ground-contact/no-fly charge** (or a
  height-based termination) — NOT a further RND variant, NOT yet the
  BC-kickstart last resort. Whichever cycle reads rnd100/rnd10-cont1
  should check `env/walk_speed`, `height_err_end_mm`/`roll_peak_deg`
  (the belly-sit signature) and per-leg contact duty, not just
  gait_valid/freeprog, to tell a genuine unfreeze from a fourth
  reproduction of the same escape hatch.

## Key facts

- The RAW kawawa2022 reward stack was bank-REFUTED on 08-23: park
  (+387) out-earned clean walking (+325) under the walk goal alone —
  flat1's walk was misaligned even before the multi-goal diet starved
  it. v2e re-pricing measured: gait +346 > stall -31 > park -352 >
  sideways -609 > reverse -741 > skate -1058 > topple -1164.
- The harsh SLIPWALK doses (idle 20 / loadslip 6 / gait_gate) are
  refuted for from-scratch discovery (8 statue arms, amp track).
- Lost code: desktop temp commit b126ceb3 (RecurrentPPO/LSTM trainer
  support) was never pushed and the pod deploy copy was overwritten;
  recipe/tests/docs were recovered from the pod and re-landed
  canonically with `--activation-fn` trainer support (08-23).

## WAITING-ON

(none)

## Now (updated 08-23 ~22:5x — RND coef=0.02 replicate read)

- **`cw-walkcurr-pf-fwd6-rscale50-rnd1` FAIL (verdicted):** this arm
  turned out to be a concurrent-cycle REPLICATE of `fwd6-rnd02` (same
  rscale50 base, same `--rnd-coef 0.02`, same 2M, independent init) —
  and it fails identically, so the coef=0.02 read is now
  replicate-confirmed, not an init fluke. Gate 0/6 det+sto, all six
  legs sacrificed, fwd 0.02 m, identical static splayed crouch on
  video. W&B mirrors rnd02: clip_fraction healthy throughout
  (0.014-0.060), value_loss tamed (310->0.4),
  `walk_freeprog_score` -0.105 -> -0.0086 monotone (best-ever rung-1
  band, tied with rnd02's -0.007) but NO zero-crossing;
  `rnd/intrinsic_mean` decays 0.037->0.0055 exactly as designed.
  Aligned-but-undertrained-at-dose per 08-21, matching rnd02/rnd10.
  **RND wave state: coef 0.02 FAIL x2 (replicated), 0.10 FAIL;
  0.06 (`rscale50-rnd3`), 1.0 (`rnd100`), and the +4M budget fork
  (`rnd10-cont1`) all FINISHED awaiting triage — those three reads
  decide bigger-dose vs the last-resort BC-kickstart. Do NOT fund
  further RND arms before they read; the coef=0.02/0.10 doses are
  closed at 2M.**

## Now (updated 08-23 ~23:0x — dose axis CLOSED, escalate to reward-shape, not more RND)

- **`cw-walkcurr-pf-fwd6-rnd100` FAIL (verdicted): the 50x dose point
  closes the RND-dose axis outright.** `--rnd-coef 1.0` (vs 0.02/0.10)
  genuinely moved the optimizer harder than any prior rung-1 arm
  (`clip_fraction` 0.12-0.17, the highest ever logged on this
  lineage, vs 0.03-0.08 at the lower doses) — so this was NOT another
  dose-insensitive no-op — yet it still lands 0/6 det gait_valid, all
  six legs sacrificed, and `walk_freeprog_score` ends WORSE (-0.020)
  than both lower doses (-0.005/-0.007). **`height_err_end_mm` is
  116.3mm on every det episode, EXACTLY matching rnd02, rnd10, the
  concurrent replicate `fwd6-rscale50-rnd1`, and (on a totally
  different rung-0 sub-goal diet) `rung0-swing3-rnd1`/`-rnd3`** — six
  independent arms, 3 RND doses spanning 50x, 2 different task diets,
  one identical collapse height. More optimizer churn at 50x dose
  still can't escape the basin — this rules out "exploration wasn't
  strong enough" as the blocker.
  **CONCLUSION: the RND-dose axis (0.02/0.10/1.0) and the rung-0
  sub-goal axis (swing-income 1x/3x, +RND 1x/3x) are BOTH closed —
  8 arms total this cycle/adjacent cycles, every one converging on
  the identical belly-sit pose regardless of mechanism or dose. Per
  the pre-registered order this file already named, the next step is
  NOT a bigger RND dose and NOT yet the BC-kickstart last resort —
  it is a **reward-shape dig-in**: find and reprice/re-terminate
  whatever lets a low, splayed, near-zero-duty pose at height error
  ~116.3mm collect enough BASE (non-walk-specific) reward to beat
  every explored alternative (candidates already named: a direct
  ground-contact/no-fly charge, or a height-based termination —
  `k_height=2.0` alone is evidently too cheap at this reward scale
  to price the pose out). DIG-IN flagged this cycle (triage-tier) for
  the deep model to root-cause height_err_end_mm=116.3mm precisely
  (what termination/kernel geometry makes that specific height
  stable) and bank-prove a fix before any further rung-1 or rung-0
  training spend.

## Now (updated 08-23 ~23:1x — rnd10-cont1's budget-axis read is DIFFERENT from every other arm; NOT folded into the reward-shape DIG-IN above — new DIG-IN flag)

- **`cw-walkcurr-pf-fwd6-rnd10-cont1` UNVERDICTED, DIG-IN (do not
  fold into the belly-sit/height_err_end_mm=116.3mm consensus above
  without reading this first).** This is the +4M budget continuation
  of `rnd10` (same `--rnd-coef 0.10`, same everything, warm-started
  from rnd10's own checkpoint) — the "budget, not dose" arm this
  file's own 22:5x section said to wait for. **Its own-cfg DR-0 gate
  is qualitatively UNLIKE every one of the 7+ belly-sit arms
  (rnd02/rnd10/rnd100/rscale50-rnd1/rung0-swing3-rnd1/-rnd3 etc):
  `gait_valid: true` and `sacrificed_legs: []` on ALL 6/6 det
  episodes** (first time any rung-1 or rung-0+RND arm has cleared
  gait_valid at all), with real per-episode telemetry: `speed_mean_m_s
  0.054` (essentially ON the 0.05-0.06 commanded band), `forward_dist_m
  0.056m`, non-degenerate `duty_cycle` per leg ([0.35, 0.29, 0.94,
  0.16, 0.42, 0.97] — imbalanced but not the frozen/near-zero-duty
  belly-sit signature), `swing_count` 1-3 per leg. **But it still
  reads `success: false` on every episode: all 6 det episodes
  terminate `tilt_pitch` (`roll_class: "fell"`) after roughly ~1s of
  real walking** (back-computed from forward_dist/speed), and
  `direction_err_mean_deg` is 52.7deg (over whatever cap the gate
  uses) with `slip_per_m` 4.51 (over the ~3 cap). Sto mode is mixed:
  2/6 also clear gait_valid (one also tilt_pitch-terminates after
  real motion), the other 4/6 sacrifice a leg (closer to the belly-sit
  family). **W&B history shows this arm is NOT monotonically
  approaching a walking optimum either**: `env/walk_freeprog_score`
  improved fastest early in the +4M extension (best at ~25% through,
  -0.0126) then WORSENED over the back half to -0.046 by the end
  (worse than its own quarter-1 read) while `env/walk_speed` bottomed
  at 0.0059 m/s around the same point and only partially recovered to
  0.0185 m/s by the end — i.e. training moved through some better
  intermediate state and partially regressed. **A `_best`-tagged
  checkpoint exists on-pod from `--best-ckpt`
  (`ppo_goal_cw_walkcurr_pf_fwd6_rnd10_cont1_best.zip`, saved ~3 min
  before the final checkpoint) and has NOT yet been evaluated** — this
  cycle pulled it to the controller and kicked off its own-cfg det+sto
  eval directly on `hexapod-mjx-train-2` via `kubectl exec`
  (`/tmp/eval_rnd10_cont1_best.log` on that pod, `--out
  logs/ckpt_eval/cw_walkcurr_pf_fwd6_rnd10_cont1_best_gate`) but it was
  still running (CPU eval, MJX compile overhead) when this cycle had
  to hand off — check that log/output dir first before re-running.
  **Why this matters for the reward-shape DIG-IN above**: that
  synthesis (from `rnd100` + 5 other belly-sit arms) concludes the
  blocker is a reward-shape/termination fix (price the low-height
  pose out directly) BEFORE any more dose/budget spend — but this
  arm, on the EXACT SAME reward shape as rnd10/rnd100, at the SAME
  dose as rnd10 with only +4M more budget, produced the only
  real-gait/real-progress episodes seen anywhere on this track,
  immediately followed by a NEW failure mode (pitch-topple after ~1s)
  that none of the belly-sit arms exhibit. Two readings compete: (a)
  this is genuine evidence budget (not reward-shape) was the
  remaining blocker, and the current checkpoint is a transient near a
  basin boundary that either more budget or a stability fix (term
  cause: pitching forward — check whether `k_pitch`=0.2 is
  underpriced relative to forward-progress income, or whether the
  policy is lurching rather than a controlled gait) would push
  through to real walking; OR (b) this is a fragile one-off (a single
  seed's brief excursion out of the belly-sit basin that the reward
  landscape doesn't actually reward keeping) and the back-half
  regression (freeprog -0.013->-0.046) is early evidence it's already
  collapsing back. **Distinguishing these is exactly a DIG-IN task**:
  read the `_best` checkpoint eval once it lands, watch the full
  video (not just contact-sheet frames) for `walk_det_0.mp4` to see
  whether the fall looks like a controlled-forward-lurch-then-trip or
  a random flail, check whether a SECOND +4M continuation from
  THIS checkpoint (not a fresh rnd10-cont2 from scratch) trends
  further away from or back toward real walking, and only then decide
  whether to fund a `k_pitch`-focused stability follow-up ahead of, or
  parallel to, the reward-shape/height-termination fix already
  flagged. Do not close the budget axis as "also refuted" off the
  `rnd100` dose-only read — this is a different arm with a materially
  different result.
  Evidence: `logs/ckpt_eval/cw_walkcurr_pf_fwd6_rnd10_cont1_gate/
  report.json` (+ contact sheet + per-episode mp4s), W&B run
  `i5d3p177`, checkpoints `ppo_goal_cw_walkcurr_pf_fwd6_rnd10_cont1{,
  _best}.zip` (both pulled to the controller).
  **UPDATE, same cycle — the `_best` checkpoint eval finished and
  sharpens the picture: `_best` reproduces the belly-sit signature
  EXACTLY (`height_err_end_mm=116.3mm`, all 6 det legs sacrificed,
  speed 0.005 m/s — genuinely indistinguishable from rnd02/rnd10/
  rnd100), while the FINAL (non-best) checkpoint is the one with the
  real gait_valid/forward-motion/pitch-fall episodes.** Since SB3
  `--best-ckpt` selects on accumulated episodic reward and the belly-
  sit pose survives the full 25s collecting a steady small per-step
  income while a walking-then-falling episode gets truncated early
  (losing the rest of that income and eating `term_penalty=24`), the
  reward function is currently pricing "survive motionless" strictly
  above "attempt to walk, risk a fall" — i.e. **this is itself
  reward-shape evidence, not a separate axis from the belly-sit
  DIG-IN above**: the SAME term_penalty/survival-income imbalance
  that makes belly-sit a stable attractor is also plausibly what
  drags the tail-end policy back toward it after episode 22:5x's
  brief walking excursion (freeprog's back-half regression -0.013->
  -0.046). Recommend the reward-shape DIG-IN explicitly check whether
  softening `term_penalty` relative to per-step survival income (or
  paying an explicit small per-step walking-attempt bonus that
  survives a fall) unlocks the walking excursion seen here, before
  or alongside the height/ground-contact charge fix.

## Now (updated 08-23 ~23:2x — height-gate/termination mechanism built + 2-arm dose grid launched)

- **`cw-walkcurr-pf-fwd6-rnd100` FAIL (verdicted, this cycle):** closes
  the RND dose axis at the top end (0.02/0.10/1.0, a 50x span, zero
  qualitative difference) — same belly-sit collapse (`height_err_end_mm`
  115-116mm, roll_peak 2-7deg, rock-stable not falling), clip_fraction
  healthy and RISING (0.014->0.167, no optimizer crush at any dose),
  but `env/walk_speed` actively COLLAPSES 0.101->0.007 m/s over the
  run. RND-as-a-class is refuted across dose; `rung0-swing3-rnd1`/
  `-rnd3` were already verdicted FAIL (belly-sit again) by a concurrent
  cycle before this one read them.
- **MECHANISM BUILT (this cycle): the belly-sit escape hatch is closed
  by composing two ALREADY-EXISTING, previously-unused-on-this-track
  levers** — `reward.walk_height_gate` (Gaussian-gates walk income by
  height error; first proven on the unrelated `cw-dynrep-criticD-
  walkcurr4` lineage 08-18, `WALKCURR4` bank in
  `test_task_semantics.py`) + `safety.walk_max_height_drop_mm` /
  `walk_height_grace_s` (opt-in belly-collapse termination, already
  wired in `sim_env.py::_step_finish`, never enabled on any walkcurr
  arm to date). Root cause (confirmed by direct reward-trace probe,
  eval-only, no training cost): under the x0.02-scaled rung-1 stack
  the belly-sit pose's only charge (env.py's quadratic `k_height`,
  scaled to 2.0) is tiny (-0.03/tick at 116mm) next to the discovery-
  friction charges it dodges by never loading a foot, and NOTHING
  ends the episode early — the pose rides the full 25s collecting a
  small-but-positive-relative-to-trying income. This is the same
  mechanism the concurrent `rnd10-cont1` `_best`-vs-final-checkpoint
  finding independently pointed at (survive-motionless out-earning
  attempt-and-risk-falling because a fall forfeits remaining income +
  eats `term_penalty` while belly-sit forfeits nothing) — converging
  evidence for the same fix.
  **Calibration**: `calibrate_walk_height.py` run fresh this cycle
  against the honest scripted tripod gait (12s x3 seeds x3 directions):
  rides `+0.4..+6.0mm` around start (mean +3.8, std 1.5) — meaning
  even a loose sigma/drop has 2.5-10x headroom over real gait noise.
  A quick stance sweep (`park`-style static hold at increasing sim-
  relative knee angle) found hip=20/knee=155 settles at -109.8mm,
  matching the observed -116mm collapse almost exactly — used as the
  new `belly_sit` scripted twin.
  **Bank landed**: `test_walkcurr_pf_hgt_*` (4 tests x 2 doses = 8
  cases, all green) in `test_task_semantics.py`, plus a `belly_sit`
  policy option added to `_slipwalk_rollout` (hip=20/knee=155 static
  hold). Proves, under the exact x0.02 rung-1 stack: gait/park/stall
  all clearly out-earn belly_sit once gated, the honest gait keeps
  >=90% of its ungated income (sigma not miscalibrated), the
  pre-existing v2e ranking (gait>>park/stall) is undisturbed, and the
  safety cutoff actually fires (belly_sit's episode cut from 375
  steps to ~43-50 vs riding to truncation ungated). Full suite
  re-run clean: 214 pass / 1 pre-existing-unrelated red (`fastprof`,
  untouched, documented track record) / 4 skip / 1 xfail.
  **CAVEAT surfaced by the bank itself** (recorded honestly, not
  gate-blocking): comparing raw RETURNS gated-vs-ungated for the same
  behavior is confounded by the probes' synthetic 1s zero-command
  hold (a fixed, large-relative-to-a-short-terminated-episode income
  artifact, same one the scaled bank already documents as a
  training-irrelevant probe-only quirk) — the bank therefore checks
  steps-to-termination and cross-behavior (gait/park/stall vs
  belly_sit) comparisons, not raw same-behavior gated/ungated deltas.
- **LAUNCHED, 2-arm dose grid (batched per operator 08-22 ruling —
  both bank-proven before either launched):**
  `cw-walkcurr-pf-fwd6-hgt1` (loose dose: `walk_height_sigma_mm=15`,
  `walk_max_height_drop_mm=60`, `walk_height_grace_s=1.5` — picked
  from-scratch-friendly, 2.5x the honest gait's calibrated band)
  and `cw-walkcurr-pf-fwd6-hgt2` (tight dose: `sigma=11`/`drop=25`/
  `grace=2.0`, matching WALKCURR4's proven-elsewhere calibration
  exactly). Both respec'd off `cw-walkcurr-pf-fwd6-rscale50` (fresh
  discovery, 2M, NOT warm-started), both VERIFIED RUNNING
  (train-1, train-0). Decision: whichever dose (or neither) crosses
  `walk_freeprog_score` past 0 with real six-leg stepping on video
  decides the mechanism's operating point before any rung-2 respec;
  if BOTH still show the belly-sit signature (same `height_err_end_mm`
  band), the mechanism class itself is insufficient and the next
  escalation is a direct minimum-total-foot-contact charge (last
  fallback before BC-kickstart). Watch specifically for a SHALLOWER
  static pose just under the drop threshold (e.g. 40-59mm on hgt2) —
  that would mean the cutoff needs tightening further, not abandoning
  the mechanism.

## Now (updated 08-23 ~23:5x — hgt2 FAIL + rnd10-cont1 FAIL, RND-as-a-class fully closed; park_duty/grace confound found)

- **`cw-walkcurr-pf-fwd6-hgt2` (tight dose) FAIL (verdicted this
  cycle):** `walk_freeprog_score` never leaves [-0.12,-0.08] across
  the full 2M, `ep_rew_mean` flat at 14.0 after the first quarter
  (24.1/14.0/14.0/14.0), `clip_fraction` crashes near-zero mid-run
  (0.0035->1e-5->5e-5, only ticks back to 0.014 at the very end) —
  reward AND eval both flat, genuinely stuck per the 08-21 ruling.
  The termination DOES fire exactly as designed: 12/12 det+sto
  episodes terminate `walk_low_height` at/near the 2.0s grace
  boundary (`height_err_end_mm` ~100mm vs the 25mm threshold, video
  confirms a fast crouch-to-belly within ~1s) — the safety cutoff
  mechanism works, but alone it does not unlock walking exploration.
  **NEW CONFOUND (own-cfg wandb read, no extra training cost):**
  `walk_height_grace_s=2.0` exactly equals the default
  `goal.park_duty_window_s=2.0` — the per-leg contact-duty history
  buffer never fills before termination fires, so
  `env/reward_park_duty` is **EXACTLY 0 for the entire run**. The one
  pre-existing charge that already prices a permanently-airborne/
  never-loaded leg (the `duty<0.1` branch of `k_park_duty`) never
  gets a chance to act on this behavior under this dose. hgt1's grace
  (1.5s) is shorter still, so the same buffer-never-fills confound
  likely applies there too — worth checking once hgt1 is triaged.
- **`cw-walkcurr-pf-fwd6-rnd10-cont1` FAIL (verdicted this cycle) —
  closes the RND budget axis; combined with `rnd100`'s already-closed
  dose axis, RND-as-a-class is now FULLY REFUTED at any bank-safe
  dose or budget, before BC-kickstart.** Reward quarters rose then
  regressed (59.9/63.1/51.4/37.9 — peaked ~25% through, worsened over
  the back half). `--best-ckpt` (selects on accumulated episodic
  reward) reverts to the track's familiar belly-sit signature
  (`height_err_end_mm=116.3`, all 6 det legs sacrificed, 0.005 m/s).
  The FINAL checkpoint shows a genuinely NEW failure mode: a forward
  lurch with real `forward_dist` (~0.056m) and `gait_valid=True` on
  all 6 det episodes, but terminates `tilt_pitch` 6/6 det (+ several
  sto) every time — video shows legs splaying and the body pitching
  forward/down within ~1s, a controlled fall, not alternating stance/
  swing cycling. Mechanism: a fall forfeits the rest of the episode's
  income and eats `term_penalty`, while belly-sit collects a small
  steady income for the full 25s — reward prices "survive motionless"
  strictly above "attempt to walk, risk a fall." This is the SAME
  survive>risk-a-fall reward-shape finding that already motivated the
  height-gate mechanism — independent corroboration, not a new axis.
- **Net position, this cycle:** every previously-open escalation
  branch except height-gate is now closed (RND: dose+budget both
  refuted; rung-0 swing income: closed; GRU: closed; gSDE: closed;
  rscale dose+continuations: closed). The height-gate mechanism
  (hgt1/hgt2) is the only branch still live, and hgt2 (tight dose)
  alone does not clear it. **hgt1 (loose dose) finished training on
  wandb (`state=finished`, `bmj5oyfo`) but was explicitly out of
  scope this cycle ("still training, leave its pod alone") and has
  not yet been prestaged/triaged — next cycle should read it, check
  it for the SAME park_duty/grace confound, and only then decide
  between (a) a park_duty_window/grace recalibration respec of the
  height-gate mechanism, (b) escalating to a direct minimum-total-
  foot-contact charge (new mechanism, needs its own scripted twin +
  bank proof before launch), or (c) finally flagging BC-kickstart to
  the operator per item (d).** No further walkcurr training was
  launched this cycle: the one live decision point is this joint
  read, and inventing a parallel arm ahead of it would pre-empt the
  track's own pre-registered sequencing rule.

## Now (updated 08-24 ~00:2x — hgt1 confirmed, park_duty-class CLOSED at every dose, DIG-IN flagged)

- **`cw-walkcurr-pf-fwd6-hgt1` (loose dose) FAIL, verdicted (a
  concurrent cycle raced this same triage; both readings agree, no
  conflict — recorded here for the doc trail):** clip_fraction stays
  healthy/rising the whole run (0.0125->0.088->0.112, no crush) yet
  reward monotonically FALLS (ep_rew_mean quarters 32.0/14.3/14.0/
  13.5/10.5) while `env/height_err_mm` climbs monotonically
  11.5->43.6mm — the policy is actively learning to crouch lower
  every checkpoint, converging TOWARD (not away from) a static
  optimum, just never crossing the loose 60mm cutoff in 2M. Own-cfg
  gate: det panel is a **rigid tripod-lock** — duty
  `[0.96, 0.98, 0.03, 0.02, 0.99, 0.03]` on every one of 6/6 episodes
  (3 legs permanently planted at ~1.0 duty, 3 permanently airborne at
  ~0.03), `sacrificed_legs=[2,3,5]`, `gait_valid=false`,
  `forward_dist_m≈0.02`, `height_err_end_mm≈41.4` (consistently just
  under the 60mm cutoff — video-confirmed, walk_det_0.png: a static
  three-legs-down/three-legs-up stand, zero body translation across
  all 10 frames). sto mode is noisier and worse: 4/6 episodes
  `tilt_pitch`-terminate attempting to escape the hold, slip/m up to
  38.8. This is exactly the pre-registered **"NEW shallower static
  pose just under the drop threshold"** branch, and closes the loose
  end of the height-gate dose bracket the same way hgt2 closed the
  tight end.
- **`cw-walkcurr-pf-fwd6-hgt2-pdw05` / `-pdw05-pdx15` FAIL (both
  verdicted this cycle) — the park_duty/grace confound fix WORKS
  mechanically but does not change the outcome, at either bank-legal
  dose.** Both are single/double-lever respecs of hgt2 fixing the
  `goal.park_duty_window_s`==`safety.walk_height_grace_s` confound
  hgt2 found (window shortened 2.0s->0.5s so the trailing-duty buffer
  fills before termination), one at unchanged `k_park_duty=0.08` and
  one at `0.12` (1.5x — the largest dose the bank allows; 3x was
  tried and REFUTED at the bank stage first, since it over-taxes the
  honest `park` stand-still behavior itself and inverts
  `park_gated`/`belly_sit_gated`, `test_walkcurr_pf_hgt_*` now 16/16
  green across 4 parametrized doses incl. both of these). Both
  launched, trained (2M, ~215s each — MJX throughput is fast enough
  that a same-cycle dose pair completes within one triage session),
  and read this cycle. `env/reward_park_duty` is confirmed nonzero
  throughout on both (was EXACTLY 0 on hgt2) — the fix lands — but
  `env/walk_freeprog_score` stays flat in [-0.11,-0.10] on both
  (ends -0.098/-0.101, no better than hgt1/hgt2), `env/walk_speed`
  decays 0.10->0.054 m/s on both (nearly identical curves), and
  `env/height_err_mm` climbs 11->51mm on both. Own-cfg gate: 6/6 det
  + 6/6 sto terminate `walk_low_height` on both arms, `gait_valid`
  is now **TRUE** on every episode (duty spread 0.12-0.84, no more
  sacrificed legs — the confound fix DOES close the specific
  tripod-lock escape) but `forward_dist_m` is 0.005-0.03/25s and
  `slip_per_m` 5.8-7.8 on both — video (`walk_det_0.png`, both arms,
  visually near-identical) shows a **slow progressive splay-and-sink**
  over the episode into a wide low crouch, zero net translation,
  ended by the safety cutoff. **Reading: de-confounding the charge
  changes WHICH static pose the policy settles into (rigid
  tripod-lock -> gradual splayed sink) but not WHETHER it settles
  into one — the survive-motionless-beats-risk-a-fall economics this
  entire campaign has been probing is untouched by this lever, and
  1.5x dose is no different from 1x (the pre-registered "tied"
  branch).**
- **TRACK STATE: every named escalation lever is now closed.** RND
  (dose 0.02/0.10/1.0 + budget continuation), rung-0 swing-income
  (1x/3x), `--gru`, `--use-sde`, reward-scale dose+continuations
  (x0.1/x0.02), height-gate loose/tight dose, and now park_duty
  confound-fix at both bank-legal doses — eight independent mechanism
  classes, all refuted with aligned reward+eval (reward flat or
  actively worsening, never crossing into real locomotion, at
  adequate 2M-6M budgets). What remains from the track's own
  escalation order: **(b) a direct minimum-total-foot-contact charge
  is the one UNBUILT mechanism** (price ANY near-static pose
  directly — e.g. a charge on "too few legs cycling within a trailing
  window" that doesn't key off height or per-leg duty extremes the
  way `k_park_duty` does, so it can't be dodged by finding a new duty
  *pattern* the way tripod-lock -> splayed-sink just did) — needs its
  own scripted twin (the splayed-sink pose specifically, not just
  `belly_sit`/`park`) and bank proof before any launch; **(d) BC-
  kickstart** is the operator-flagged last resort. **DIG-IN flagged**
  for whichever cycle picks this up next: design+build+bank-prove the
  foot-contact charge (preferred, stays inside the "no BC teacher"
  rule) unless the deep read concludes the reward-shape space is
  structurally exhausted, in which case flag BC-kickstart to the
  operator explicitly rather than trying a ninth reward variant.
  Evidence: `logs/ckpt_eval/cw_walkcurr_pf_fwd6_{hgt1,hgt2_pdw05,
  hgt2_pdw05_pdx15}_gate/`.

## Now (updated 08-24 ~00:3x — ROOT CAUSE FOUND: raw-joint action-space zero point is a belly-sit pose, by construction, no policy required)

- **DIG-IN (this cycle, triaging `cw-walkcurr-pf-fwd6-hgt2-pdw05` —
  concurrent cycle already verdicted it + its `-pdx15` sibling FAIL
  above; this entry does not re-verdict, it adds a finding neither
  read had): the report.json metrics for hgt2-pdw05/hgt2/hgt1
  CONTRADICT the "frozen static crouch" framing used across most of
  the last ~20 verdicts.** `walk/det` episodes show real
  `swing_count` (4-11 per leg, not 0), duty spread 0.12-0.84 (not
  pinned), and `speed_mean_m_s` 0.055-0.065 -- close to the 0.05-0.06
  m/s COMMAND. What actually fails is `direction_err_mean_deg`
  ~80deg (should be ~0) and `slip_per_m` ~7.3-7.5 (cap 3.0): the legs
  ARE cycling, just not coherently enough to produce net forward
  travel before the height-gate terminates the episode ~2s in. Coarse
  6-frame video sampling at ~0.5s intervals aliases this real
  high-frequency limb activity into what reads, by eye, as a static
  splayed pose.
  **ROOT CAUSE, found by a zero-training, zero-reward physical probe
  (no policy at all -- just `env.step(np.zeros(18))` in a loop):**
  this recipe's action space is `SimHexapodJointGoalEnv` (raw 18-dim
  joint targets, `--task joint_walk`), where `a=0` maps to
  `action_to_q_rad(0)` = the HARDWARE AXIS MID-RANGE (hip=-25deg,
  knee=65deg per leg, from `_CENTER_RAD` in `joint_task.py`) -- NOT
  the settled standing pose. The env's own post-reset `q_nom` (read
  from the physics-settled stand) is hip~15.75deg/knee~84.8deg,
  matching the semantics bank's `WALK_PLANT=(20,80)` almost exactly.
  Stepping the env with a CONSTANT all-zero action for 50 ticks (2s,
  no policy, no reward signal driving it) sinks the chassis
  **-110mm** while roll/pitch stay **exactly 0.00deg** the entire
  time -- bit-for-bit the "belly_sit" signature (`height_err_end_mm`
  ~110-116mm, level attitude) that EVERY RND arm (rung-0 1x/3x,
  rung-1 0.02/0.10/1.0 dose, +4M budget), both height-gate doses, and
  both park_duty-confound-fix doses independently converged to. A
  freshly initialized (small-weight, near-zero-mean) PPO policy
  reproduces this basin by construction -- none of the 8+ "closed"
  mechanism classes (RND, rung-0 swing-income, GRU, gSDE, reward
  rscale dose/budget, height-gate dose, park-duty dose) could have
  fixed this, because none of them touch what the action space's
  neutral point physically does. This is the "sim/action defect"
  link in the root-cause chain the 08-21 ruling asks to check before
  another reward patch, and it was never checked until this cycle
  because every prior dig-in reasoned about REWARD shape, not the
  ACTION MAPPING.
  **FIX BUILT + TESTED (bit-exact when off):** `joint_task.py` gains
  a cfg-gated per-axis action bias (`goal.joint_action_bias_{yaw,
  hip,knee}_deg`, default 0.0 each) applied inside `_act_to_q` before
  the existing `action_to_q_rad` mapping -- a pure translation of the
  action's zero point, clipped back into the hardware axis range, so
  the full reachable range and every other joint_task consumer are
  untouched at the default. `test_joint_action_bias.py` (6/6 green):
  pins the defect (`test_zero_action_collapses_without_the_fix`,
  reproduces the -110mm/level-attitude collapse from zero action
  alone), proves bias=0 is bit-exact against the un-biased mapping
  for random actions, proves a nonzero bias shifts exactly the
  requested degrees and clips correctly at the extremes, and proves
  the SAME zero-action probe stays within 30mm of reference height
  once the bias re-centers `a=0` on `hip=20deg/knee=80deg` (+45/+15
  vs the hardware-midrange default). Full `test_task_semantics.py`
  walkcurr bank re-run clean (56/56) -- the change is additive and
  does not touch reward semantics.
  **LAUNCHING (this cycle): `cw-walkcurr-pf-fwd6-actbias1`** --
  single new lever vs the strongest existing rung-1 baseline
  (`cw-walkcurr-pf-fwd6-rscale50`, the only prior arm whose
  `walk_freeprog_score` ever trended toward 0): add
  `goal.joint_action_bias_hip_deg=45` / `_knee_deg=15`, from scratch,
  2M, everything else byte-identical. Prediction-if-true: with the
  policy's initial/undertrained mean action no longer physically
  sinking the chassis, `walk_freeprog_score` should leave the
  [-0.11,-0.05] dead band materially faster/further than rscale50
  did alone, and det video should show the robot actually STANDING
  at t=0 instead of visibly sinking over the first ~1-2s regardless
  of command. Prediction-if-false (same collapse signature despite
  a verified-correct neutral pose): the zero-point defect was real
  but not the (only) blocker -- fall back to the pre-registered
  foot-contact-charge mechanism or BC-kickstart per the entry above,
  but only after checking whether PPO's exploration noise (std~0.37
  on 18 dims) is simply large enough to random-walk back into the
  collapse basin regardless of where the mean sits (worth an
  action-bias + tighter-log-std-init pairing as the next fork if so).
  Evidence: `rl_move/sim/joint_task.py`, `rl_move/tests/
  test_joint_action_bias.py`, snapshot tag
  `exp/walkcurr-fwd6-actbias1`.

## Now (updated 08-24 ~00:4x — Stage-A tangent-slip diagnostic read: loosened safety is ACTIVELY HARMFUL)

- **`cw-walkcurr-pf-fwd6-stagea-slip1` FAIL (verdicted this cycle,
  full mechanism read in the ledger/W&B note).** The Stage-A arm
  (fixed 0.05 m/s diet + planted-foot tangent-slip charge + LOOSENED
  safety: `walk_max_height_drop_mm=0.0` i.e. height cutoff disabled,
  grace 5s, roll/pitch 35deg) made the belly basin strictly WORSE:
  instead of the static crouch/belly-sit, the policy drops FLAT onto
  its belly within seconds and ends with **5.3/6 feet airborne**
  (`walk_swinging_feet` 3.1->5.3, liftoff_count 0.38->0.10/s,
  height_err 11->111mm, speed 0.10->0.009 m/s) while reward falls
  MONOTONICALLY (43.8/38.8/25.7/9.5 -> -0.03) — the safety
  termination was evidently the only pressure bounding time-in-basin;
  removing it made belly-flat an absorbing state PPO's state
  distribution slides into even as collected reward falls. Own-cfg
  gate: det 0/6 gait_valid, ALL SIX legs sacrificed, prog -0.01, fwd
  0.01m, ZERO terminations; det video = splayed belly-flat by frame 2,
  no motion after. Three consequences for the open escalation DIG-IN
  (pdw05/pdx15): (1) any anti-collapse charge (incl. the named
  minimum-total-foot-contact/no-fly charge) must come WITH a
  termination/hard boundary, never INSTEAD of one — absorbing states
  beat prices; (2) the tangent-slip charge is CLOSED as a discovery
  lever on this rung — it is evaded by zero contact
  (`reward_foot_slip_tangent` -0.00007 all run) and returns no data in
  exactly the failure mode that matters; (3) METRIC WARNING:
  `walk_freeprog_score` read "best-ever" (-0.016 trending to 0) here
  purely because EMA speed -> 0 while lying down — never read
  freeprog-toward-zero as discovery health without a contact/height
  cross-check. This is now the ~11th independent mechanism converging
  on the same static/collapsed basin from random init; it strengthens
  the BC-kickstart side of the escalation fork the pdw05/pdx15 DIG-IN
  owns (actbias1, launched by a concurrent cycle, is the last
  in-flight zero-point/mechanism arm before that fork must be taken).

## Now (updated 08-24 ~00:5x — pdw05/pdx15 DIG-IN CLOSED; fork rule pinned)

- **DIG-IN COMPLETE (deep cycle, `cw-walkcurr-pf-fwd6-hgt2-pdw05-pdx15`
  + sibling): verdicts STAND as recorded (both FAIL, ledger + W&B +
  RL_LOG already fanned out by the 00:2x cycle — no re-verdict), and
  the root-cause depth the flag asked for is CONFIRMED independently.**
  (1) Re-read the pdx15 gate report: the pose is NOT frozen — swing
  6-12/leg, duty 0.24-0.78, speed_mean 0.063 m/s ≈ command — but
  incoherent (dir_err ~60deg, wrong-direction 38%, slip/m 7.76,
  fwd 0.011 m) while height_err climbs to 63-95mm until
  `walk_low_height` fires; matches the 00:3x contradiction finding,
  and the det contact sheet shows the same progressive splay-and-sink
  with zero translation. (2) Re-ran `test_joint_action_bias.py` 6/6
  green, independently confirming the 00:3x root cause: the raw-joint
  action zero (`a=0` -> hardware axis midrange hip=-25/knee=65) sinks
  the chassis ~110mm with level attitude by construction — the
  ~110-116mm attractor is where random init STARTS (action-map
  defect), and the survive-motionless>risk-a-fall economics are why
  PPO STAYS. No further training spend needed for the root cause.
  **FORK RULE (pre-registered here so the actbias1 triage is
  mechanical):** `actbias1` (in flight, the zero-point fix arm) is the
  discriminating read — (a) if its `walk_freeprog_score` leaves the
  dead band materially beyond rscale50's -0.015 best (ideally a zero
  crossing) with det video standing at t=0, the action-map defect was
  the missing blocker: continue that lineage, no new mechanism. (b) If
  it reproduces the collapse/static basin DESPITE the verified-correct
  neutral pose, the economics dominate: build the direct
  minimum-total-foot-contact charge (preferred, stays inside the
  no-BC rule) — WITH a termination, never instead of one, per the
  stagea-slip1 lesson (absorbing states beat prices) and with a
  splayed-sink scripted twin + bank proof before launch — and in the
  SAME entry flag BC-kickstart to the operator as the named last
  resort if that ninth mechanism also fails. No other walkcurr arms
  before actbias1 reads.
