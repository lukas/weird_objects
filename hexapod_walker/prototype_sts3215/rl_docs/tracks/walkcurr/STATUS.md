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

- **`cw-walkcurr-pf-fwd3-chargeramp` RUNNING (train-0, 2M discovery,
  launched ~18:2x):** the dense-charge ramp fix. fwd2 batch verdicted
  FAIL (double-triaged, verdicts agree): swing income was REACHABLE
  (0.065-0.089/step from step 0) but flat — +1/swing is ~60x below
  the -4.7/step charge flow, and the bank caps the dose at k<~1.7;
  term 1200->800 changed nothing (byte-similar freeze). Blocker =
  the dense charge flow itself. New mechanism
  `reward.walk_charge_ramp_steps` (+`_min_frac`, default 0.40) scales
  loadslip/park/idle/heading charges 40%->100% over 1M steps
  (term/drag-ramp contract mirror: default OFF bit-exact, eval judges
  full pricing; `test_walk_charge_ramp.py` 6/6). Floor bank (a
  concurrent cycle's `test_walkcurr_chargeramp_min_*` layer) MEASURED
  0.15 inverted the ranking (skate +131 > sideways, shuffle +228
  rest point) -> floor raised to 0.40, re-measured 3/3 green: full
  ranking + monotone positive ladder gait 439.7 > creep 327.7 >
  stall 230.1 > park 98.5 > shuffle 27.6 > sideways -142 > reverse
  -219 > skate -302 > topple -1164. No swing bonus (measured inert).
  Snapshot `exp/walkcurr-fwd3-chargeramp`.
  NOTE for the fwd2 dig-in loglines that named "init/BC-kickstart"
  as next: BC-kickstart contradicts the track charter (prior-free —
  no BC teacher, no motion prior); the charge ramp is the
  charter-compliant fix and is now in flight. If fwd3 freezes even
  at 40% charges, the next fork is init/noise (log_std) or a rung-0
  curriculum, still prior-free.

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
