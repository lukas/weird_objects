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

## Next

- If fwd4 (either arm) gates or shows `walk_freeprog_score` trending
  toward 0 (immature-but-real progress): continue/harden that arm;
  rung 2 (small heading set) respec once a rung-1 pass lands. Consider
  `--gru --gru-hidden-size 64` (in-repo recurrent path; the paper's
  LSTM(64) trainer support died with the unpushed desktop clone) once
  commands start changing mid-episode (rung 4), and a paper-pure
  proprioception-only obs A/B (`goal.walk_obs_body_vel=0.0`) once the
  policy is recurrent.
- If fwd4 also freezes identically: build the loadslip-bootstrap
  mechanism (soften `k_loadslip_excess` only for an early window,
  e.g. first 200-500k steps, distinct from — and re-proven against —
  the WALKCURR_PF bank at its own schedule, since the rejected 0.15
  floor was tested as a permanent-through-convergence value, not a
  short bootstrap) before declaring the prior-free MLP recipe refuted
  at this budget.
- If fwd1-lineage fails WITH aligned reward across ALL of init/noise
  + loadslip-bootstrap (bank green, reward and eval agree, adequate
  budget): the prior-free MLP recipe is refuted at this budget —
  escalate to a dig-in before any architecture/budget escalation; do
  NOT seed-sweep further reward-magnitude variants.

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
