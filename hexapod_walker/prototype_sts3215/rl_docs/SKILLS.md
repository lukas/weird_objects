# SKILLS — what the robot can do today, and which checkpoint does it

The answer to "are the successes getting lost?" (operator, 08-09).
Every PASSED capability lives here with its checkpoint. Verdicts stay
in the ledger / `rl_docs/runs/`; this is the accumulating INVENTORY.

**UPDATE RULE (binding, 08-09): a cycle that verdicts a PASS adds or
updates one row here in the same cycle.** Checkpoints are durable in
W&B artifacts (`ckpt-<name>`, type `policy-checkpoint`) — trainer
publishes automatically since 08-09; earlier ones backfilled. The
controller's `rl_move/sim/policies/` is a cache, NOT the archive
(it is gitignored and the controller is an ephemeral pod).

Demo any row locally:
`.venv/bin/python -m rl_move.sim.drive_policy rl_move/sim/policies/<ckpt>.zip`

## Walk (main line)

| Skill | Checkpoint (artifact `ckpt-<name>`) | Evidence | Envelope / limits |
|---|---|---|---|
| **CHAMPION: forward walk, 30 s, correct speed** | `ppo_goal_cw_walk_longdist_r2` (md5 bcddc65c) | c44 promotion, seed-confirmed (s1): DR0 det 6/6, slip/m 0.94–0.96, 1.63 m @ 30 s, prog 0.98 | fwd ±45° cmds only; slip ~1/m (not hardware-ready); sto stalls on some draws (dr10 arm attacking this) |
| Steer through direction changes + stops | `ppo_goal_cw_walk_wander` | c45 PASS: ±45°/5 s resample, 15% stops, gv 12/12, 0 term, prog ~1.0 through changes | fwd hemisphere ONLY — backward command = fall (operator repro'd, 08-09); change-eps slip ~2× straight |
| Crouch walking (−20 mm height) | `ppo_goal_cw_walk_lowgait` | c45 PASS: gv 12/12, 0 term, end-height err ~4 mm | slip ~champion; one sto in-place-paddle ep (lineage brittleness) |

## Stance / posture (older line — see archive for full state)

- Rise/lower heights at DR 1.0: solved pre-walk-campaign (see
  `archive/RL_PLAN_FULL_2026-08-09.md`); lower-line rework per rulings.

## Pending verdicts that would add rows

wander30/lowgait30/lowgait40 (envelope extensions), endur60(+s1)
(60 s endurance), strafe (±90°), backforth (reverse), terrain05/10
(rough ground), standwalksit (skill chaining), pose-track.

## Consolidation status (single deployable policy)

Skills above are SEPARATE checkpoints. The deployable robot needs
either one multi-skill policy (goal-mix training — `standwalksit` is
the first chaining probe) or a deploy-time skill switcher. Champion
strategy: the champion is the BASE the walk line breeds from; skill
passes are preserved here and folded in via goal-mix arms — a
promotion never deletes a skill checkpoint (append-only).
