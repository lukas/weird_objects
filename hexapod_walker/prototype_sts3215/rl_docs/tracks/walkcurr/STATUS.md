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

- **Rung-1 discovery batch v2 RUNNING (08-23 ~17:5x):**
  `cw-walkcurr-pf-fwd2-swing` (train-0) and
  `cw-walkcurr-pf-fwd2-swingterm800` (train-1), 2M discovery each,
  fresh init, exact fwd1 recipe + `reward.k_walk_swing=1.0`
  (direction-free per-completed-swing credit, the only income a
  random-init policy can reach); the term800 twin also drops
  term_penalty 1200->800 (largest drop keeping dying the strict
  floor) to isolate catastrophe-dominance. Bank-proven under exact
  cfgs incl. the pre-registered `shuffle` swing-farming attack
  (gait +409.9 > stall -21.5 > park -351.7 > shuffle -430 NEGATIVE >
  sideways -534.5 > reverse -675.2 > topple -1164/-764 > skate
  -1339; 6/6 new tests green, snapshot
  `exp/walkcurr-fwd2-swingbank`). Gate: fwd1's rung-1 panel +
  discovery-health markers (swing/step-event rate must leave fwd1's
  flat 0.02/step; walk_prog must leave 0.0).
- `cw-walkcurr-pf-fwd1` VERDICTED FAIL (dig-in, 08-23 ~17:4x):
  PPO froze into a tilt-safe splayed crouch — rational under v2e:
  walk_prog identically 0.0 all run, step events flat 0.02/step,
  every income channel unreachable from random init while the dense
  charge stack (loadslip 0.03m travel-floor, height, heading)
  punishes the flailing exploration must pass through; the
  "reward collapse" was ep_len growth x negative per-step rate
  (undiscounted logging artifact), with truncation bootstrap making
  freeze (~-470 discounted) rationally beat exploratory falling
  (-1200). The bank ranks scripted lifetimes and cannot see
  reachability — exploration-safety is now a named, separate
  property. Full chain in the run's W&B OUTCOME note.

## Next

- If fwd1 gates: rung 2 (small heading set) respec; consider `--gru
  --gru-hidden-size 64` (in-repo recurrent path; the paper's LSTM(64)
  trainer support died with the unpushed desktop clone) once commands
  start changing mid-episode (rung 4), and a paper-pure
  proprioception-only obs A/B (`goal.walk_obs_body_vel=0.0`) once the
  policy is recurrent.
- If fwd1 fails WITH aligned reward (bank green, reward and eval
  agree, adequate budget): the prior-free MLP recipe is refuted at
  this budget — escalate to a dig-in before any architecture/budget
  escalation; do NOT seed-sweep.

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
