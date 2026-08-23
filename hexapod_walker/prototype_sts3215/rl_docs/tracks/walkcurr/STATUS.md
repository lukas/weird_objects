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
