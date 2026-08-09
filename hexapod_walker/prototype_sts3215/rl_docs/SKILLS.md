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
| Steering robust to physics variation (DR 0.5) | `ppo_goal_cw_walk_wander_dr05` (md5 18af118f) | this cycle PASS: own-cfg DR0.5 gv 12/12, 0 term, prog med 0.95–0.96, slip/m med 1.39–1.70; DR0 retention gv 6/6 | ±45° cmds, gentle 5 s resamples only (abrupt-flip hardening in flight); paddle gait, not hardware-ready |
| Strafe ±90°, robust to physics variation (DR 0.5) | `ppo_goal_cw_walk_strafe_dr05` (md5 cb178b91) | this cycle PASS: own-cfg DR0.5 gv 12/12, 0 term, prog med 0.92–0.97, slip/m med 1.89–2.00; DR0 retention prog 1.09, slip 1.80 (< parent 2.20) | lateral paddle; fixed commands (no resampling trained); not hardware-ready |
| Drive anywhere in the front half-circle (±90°, resampled cmds + stops) | `ppo_goal_cw_walk_head90` (md5 bcf474ff) | c49 PASS: own-cfg DR0 gv 12/12, 0 term, lateral err ≤1.6× fwd; JOYSTICK GATE PASS @DR0.2 (0 falls incl. instant-flip stress) | prog ~0.84 on mixed headings (lateral costs progress); left strafe ~½ the displacement of right (L/R asymmetry); paddle gait, not hardware-ready |
| Crouch walking (−20/−30/−40 mm height) | `ppo_goal_cw_walk_lowgait` / `_lowgait30` / `_lowgait40` | c45/c47/c48 PASS at each rung: gv 12/12, 0 term, end-height err ≤7 mm, det agg slip/m 0.92–0.96 (≤ champion band) | envelope verified to −40 mm (−50 mm rung queued); one sto in-place-paddle ep per panel (lineage brittleness) |

## Stance / posture (older line — see archive for full state)

- Rise/lower heights at DR 1.0: solved pre-walk-campaign (see
  `archive/RL_PLAN_FULL_2026-08-09.md`); lower-line rework per rulings.

## Pending verdicts that would add rows

wander30/lowgait30/lowgait40 (envelope extensions), endur60(+s1)
(60 s endurance), backforth (reverse), terrain05/10
(rough ground), standwalksit (skill chaining), pose-track.
(strafe ±90° landed — see DR 0.5 row above.)

## Consolidation status (single deployable policy)

Skills above are SEPARATE checkpoints. The deployable robot needs
either one multi-skill policy (goal-mix training — `standwalksit` is
the first chaining probe) or a deploy-time skill switcher. Champion
strategy: the champion is the BASE the walk line breeds from; skill
passes are preserved here and folded in via goal-mix arms — a
promotion never deletes a skill checkpoint (append-only).
