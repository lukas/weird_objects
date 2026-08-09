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
| **CHAMPION: forward walk, 30 s, correct speed** | `ppo_goal_cw_walk_longdist_r2` (md5 bcddc65c) | c44 promotion, seed-confirmed (s1): DR0 det 6/6, slip/m 0.94–0.96, 1.63 m @ 30 s, prog 0.98; JOYSTICK GATE PASS @DR0.2 AND @DR0.5 (0 falls incl. flip stress; baselines: `logs/ckpt_eval/champion_longdist_r2_drive*.json`) | fwd ±45° cmds only; slip ~1/m (not hardware-ready); sto stalls on some fixed draws — NOT fixed by DR (longdist-dr05 FAIL) or resample training (stallfix FAIL); park-bank start-state arm is the live lever; backward cmd barely moves (0.06 m/7 s); L strafe 0.21 m vs R 0.30 m |
| Steer through direction changes + stops | `ppo_goal_cw_walk_wander` | c45 PASS: ±45°/5 s resample, 15% stops, gv 12/12, 0 term, prog ~1.0 through changes | fwd hemisphere ONLY — backward command = fall (operator repro'd, 08-09); change-eps slip ~2× straight |
| Steering robust to physics variation (DR 0.5) | `ppo_goal_cw_walk_wander_dr05` (md5 18af118f) | this cycle PASS: own-cfg DR0.5 gv 12/12, 0 term, prog med 0.95–0.96, slip/m med 1.39–1.70; DR0 retention gv 6/6 | ±45° cmds, gentle 5 s resamples only (abrupt-flip hardening in flight); paddle gait, not hardware-ready |
| Strafe ±90°, robust to physics variation (DR 0.5) | `ppo_goal_cw_walk_strafe_dr05` (md5 cb178b91) | this cycle PASS: own-cfg DR0.5 gv 12/12, 0 term, prog med 0.92–0.97, slip/m med 1.89–2.00; DR0 retention prog 1.09, slip 1.80 (< parent 2.20) | lateral paddle; fixed commands (no resampling trained); not hardware-ready |
| Drive anywhere in the front half-circle (±90°, resampled cmds + stops) | `ppo_goal_cw_walk_head90` (md5 bcf474ff) | c49 PASS: own-cfg DR0 gv 12/12, 0 term, lateral err ≤1.6× fwd; JOYSTICK GATE PASS @DR0.2 (0 falls incl. instant-flip stress) | prog ~0.84 on mixed headings (lateral costs progress); left strafe ~½ the displacement of right (L/R asymmetry); paddle gait, not hardware-ready |
| Crouch walking (−20/−30/−40 mm height) | `ppo_goal_cw_walk_lowgait` / `_lowgait30` / `_lowgait40` | c45/c47/c48 PASS at each rung: gv 12/12, 0 term, end-height err ≤7 mm, det agg slip/m 0.92–0.96 (≤ champion band) | envelope verified to −40 mm (−50 mm rung queued); one sto in-place-paddle ep per panel (lineage brittleness) |
| Joystick-style abrupt command flips, no falls (DR0) | `ppo_goal_cw_walk_joystick45` (md5 999bd5d6) | c49 PASS: eval_drive JOYSTICK GATE 0 in-envelope falls (fwd/diag/stop-go panel + 3 flip-stress eps); own-cfg DR0 harness gv 12/12, 0 term, prog ~1.04 | envelope heading ≤±45°, speed ≤0.06 m/s; paddle foot-slide (slip/m ~1.4-1.6, not hardware-ready); DR0.5 pair (joyjit-dr05-c1) in flight |
| 60 s endurance walking | `ppo_goal_cw_walk_endur60` | c47 PASS + c48 seed twin: both seeds ~3 m @ 60 s, gv 12/12, 0 term, NO gait decay — endurance is seed-robust (endur60's low slip 0.887 was seed luck; s1: 1.13) | anchorgate lineage (not champion); slip ~0.9–1.1/m; 1/6 sto draw-stall; champion-60s fold queued (endur60-r2) |

## Stance / posture (older line — see archive for full state)

- Rise/lower heights at DR 1.0: solved pre-walk-campaign (see
  `archive/RL_PLAN_FULL_2026-08-09.md`); lower-line rework per rulings.

## Pending verdicts that would add rows

wander30 (envelope extension), backforth (reverse), terrain05/10
(rough ground), standwalksit (skill chaining), pose-track.
(strafe ±90° landed — see DR 0.5 row above; lowgait30/40 and
endur60+s1 landed — rows above.)

## Consolidation status (single deployable policy)

Skills above are SEPARATE checkpoints. The deployable robot needs
either one multi-skill policy (goal-mix training — `standwalksit` is
the first chaining probe) or a deploy-time skill switcher. Champion
strategy: the champion is the BASE the walk line breeds from; skill
passes are preserved here and folded in via goal-mix arms — a
promotion never deletes a skill checkpoint (append-only).
