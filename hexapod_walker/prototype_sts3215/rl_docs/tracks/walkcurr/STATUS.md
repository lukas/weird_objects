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

- `cw-walkcurr-pf-fwd1` (discovery, 2M): first rung. Launched
  08-23 after the WALKCURR_PF bank went green (7/7) on the v2e
  pricing. Gate: C-env det fixed-forward panel — walk survives (zero
  tilt terms), cmd_prog_frac >= 0.35, direction_err <= 30 deg,
  slip/m <= 3.0, six legs cycling on >=4/6 episodes, video shows real
  stepping.
  **FINISHED 08-23 ~16:5x, eval run (podeval) but UNVERDICTED —
  flagging DIG-IN, not triaging blind.** W&B reward is not flat, it is
  actively COLLAPSING across training: quarter means -100 / -541 /
  -1169 / -1988 (ep_rew_mean final -2494), monotonically worse every
  quarter, full 2M budget, fps healthy (~11.9k), no crash. Own-cfg DR-0
  gate (`logs/ckpt_eval/cw_walkcurr_pf_fwd1_gate/`): det prog_frac
  ~0.00 / slip 10.36 / fwd 0.01m on ALL 6 episodes (identical numbers
  every episode -> looks like a fixed-point, not noise); sto prog
  ~0.00, slip 37-42 (one TERM tilt_pitch). Video (det + sto contact
  sheets, watched): the robot does NOT attempt to walk at all — it
  settles into a static crouched/splayed pose with the two front legs
  raised together and stays there the whole 25s episode (feet grind
  in place = the >10x-over-cap slip reading, not real stepping). Gate
  fails on every axis (prog 0.00 vs >=0.35, slip 10-42 vs <=3.0, zero
  real stepping) — this is a clean, unambiguous FAIL by the numbers,
  but the run decides the track's very first fork (is the v2e pricing
  actually PPO-exploration-safe, not just bank-ranking-safe? see next
  para) so it is being left for a dig-in cycle per the model-tiering
  rule rather than triaged and relaunched blind.
  **Working theory (not yet confirmed — dig-in should verify before
  any relaunch):** `reward.term_penalty=1200` is large relative to
  every other term in the v2e stack (gait +346 lifetime per the bank's
  own scripted-trajectory ranking); a from-scratch policy that has not
  yet discovered ANY stepping motion may find it cheaper to freeze
  into a fall-proof static pose (paying only `k_park_duty`/
  `k_walk_idle_charge`, both smaller and BOUNDED) than to explore
  stepping motions that risk the 1200 catastrophe before it has any
  gait to fall back on — an exploration/chicken-egg failure mode the
  WALKCURR_PF bank cannot see (it only ranks pre-built scripted
  trajectories against each other, never asks whether PPO can find
  the walking trajectory from random-init exploration at all). If
  confirmed, candidate fixes: a lower discovery-phase term_penalty
  with a curriculum step-up once stepping is discovered, an explicit
  step-event/exploration bonus active from step 0, or a few-thousand-
  step BC/kickstart into ANY forward stepping before switching to pure
  RL — needs the dig-in's judgment, not a same-recipe relaunch.
  Evidence: `logs/ckpt_eval/cw_walkcurr_pf_fwd1_gate/{report.json,
  contact_sheet.png,walk_det_*.mp4}`, W&B `fmbwu9p1`. Per the track's
  own binding triage rule this is exactly "reward falling + walk eval
  failing" -> no same-recipe seed sweep until the mechanism is
  audited.

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
