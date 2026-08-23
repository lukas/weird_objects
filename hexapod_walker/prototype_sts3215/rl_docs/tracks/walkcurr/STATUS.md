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

- **Rung-1 discovery batch v2 CLOSED, BOTH ARMS FAIL (08-23 ~18:0x)
  — swing-income and term_penalty-magnitude are DEAD.** UPDATE (same
  cycle, concurrent read with reward-component telemetry): the
  mechanism is now root-caused past "dig-in, direction unknown" —
  `env/reward_swing` DID pay ~0.065-0.089/step from step 0 (the swing
  lever IS reachable), but that income is ~60x smaller than the
  ~-4.7/step dense charge flow (loadslip -4.08, height -0.77, heading
  -0.5) and the bank ordering caps the direction-free swing dose at
  k<~1.7 before sideways-farming becomes profitable — so income-side
  levers (swing bonus, term_penalty magnitude) are EXHAUSTED within
  this pricing family; freezing pays MORE than exploring at any
  bank-legal dose. **Next lever named (not yet built): a
  discovery-phase CHARGE RAMP** (mirror the existing term_penalty/
  drag-allow ramp-contract pattern — cfg-armed, default OFF, anneal
  the dense per-step charges from near-zero up as training
  progresses) rather than an init/BC-kickstart change; still dig-in
  tier (a reward-schedule mechanism, root-cause chain required before
  building). Do NOT launch a 3rd single-cfg variant of this exact
  recipe before that lands. `cw-walkcurr-pf-fwd2-swing` (train-0,
  `reward.k_walk_swing=1.0`) and `cw-walkcurr-pf-fwd2-swingterm800`
  (train-1, same + `term_penalty` 1200->800) both finished in ~10 min
  (fast GPU fps) and both FAIL identically: det panel progress_ratio
  ~-0.001 all 6 episodes byte-identical, forward_dist 0.005-0.021m
  over 25s, slip/m 7.7-9.0 (cap 3.0), height_err_end 94-96mm.
  **Video (both contact sheets) shows something WORSE than fwd1's
  raised crouch: the robot starts upright for one frame then
  COLLAPSES flat onto a splayed belly-down pose and stays there the
  rest of the episode** — the nonzero swing_count in the report is
  limb micro-twitching in the collapsed pose, not gait cycling.
  `-swing`'s gait_valid even flips to False with 1 sacrificed leg
  (was True/frozen-crouch on fwd1) — the swing bonus changed WHICH
  static failure basin the policy falls into, not whether it explores
  real stepping. Reward quarters both match fwd1's monotonic-negative
  shape (expected per fwd1's own root-cause: ep_len x
  negative-per-step logging artifact, not new degradation). Per the
  batch's own pre-registered branch: since BOTH fixes froze/collapsed
  identically to fwd1, term_penalty magnitude and swing-income
  reachability are BOTH refuted as the rung-1 blocker — same
  conclusion, from two independent single-lever probes, not a shared
  confound. Bank-proven cfgs used (6/6 new tests green incl. the
  `shuffle` swing-farming attack, snapshot
  `exp/walkcurr-fwd2-swingbank`) — the failure is exploration
  dynamics, not a reward-alignment cheat the bank would have caught.
  **DIG-IN flagged for the next lever**: this decides the track's
  second fork (does rung-1 need a fundamentally different exploration
  mechanism — a few-thousand-step BC/kickstart into ANY forward
  stepping, an action-noise schedule, or an init-pose change — rather
  than another reward-magnitude dial) and per `RUN_INTERPRETATION_
  RULES.md` a genuine root-cause-chain redesign belongs to a deep
  cycle, not a same-recipe reward-tweak guess. Do NOT launch a 3rd
  single-cfg variant of this exact recipe before that design lands.
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
