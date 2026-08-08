# Hexapod RL — overnight campaign review (2026-08-08)

Self-contained brief for an outside reviewer. Covers 20 W&B runs across 5
rounds on 4 CoreWeave GPU pods, Fri Aug 7 ~6:30 PM → Sat Aug 8 ~6:45 AM.

## Context (read this first)

- **Robot:** 18-DOF hexapod (6 legs × hip yaw / hip pitch / knee), Feetech
  STS3215 serial-bus servos, Arduino Uno Q controller, IMU on body. Control
  at 25 Hz over HTTP. One knee (L5 / bus ID 19) was previously cooked by an
  unsafe stand attempt — hardware safety gates are strict.
- **Sim:** MuJoCo 3.11 twin (`rl_move/sim/sim_env.py`) with calibrated servo
  model (latency, rate limit, dead-band, torque→current estimate), hinged
  foot pads, domain randomization (DR) over geometry, mass, IMU mounting,
  friction, actuator params. DR "scale" 0.0–1.0 multiplies all ranges.
- **Action space:** raw 18-joint position targets (deltas from nominal,
  low-pass filtered). A previous 6D body-IK action space was abandoned: it
  learned flat-rise choreography but the skill was noise-fragile (survived
  only at policy std ≤ 0.05) and PPO kept rationally abandoning it.
- **Policy:** PPO (SB3), MLP, goal-conditioned. Goal modes: hold / lean /
  track / unload (per-leg) / raise (small body lift) / rise (belly→stand,
  from flat, "bridge" = partially curled, or crouch starts) / lower
  (stand→belly) / walk (commanded planar velocity). Walk uses privileged
  body velocity in obs (sim-only; deployment needs frame-stack or
  distillation later).
- **Evaluation:** every checkpoint goes through an exact-path harness
  (`rl_move/sim/eval_checkpoint.py`): 6 episodes/mode, deterministic AND
  stochastic passes, at the run's own DR level. Reports per-servo currents
  (peak, p95, hot-time above 1.5 A, cross-leg imbalance), foot duty cycles,
  gait metrics (stride, slip, swing count), success per mode, video.
  W&B scalars alone repeatedly missed regressions the harness caught.
- **Training infra:** 4 CoreWeave pods (`hexapod-sweep-{friction,s3,s4,long5m}`),
  48 parallel envs each, ~1.5–2M steps/hour. Runs warm-start from parent
  checkpoints; lineage tracked in W&B (fork_from) + `lineage.json`.

## Headline results

| Metric | Value |
|---|---|
| Stand↔belly DR ladder gates passed | 6/6 (DR 0.2 → 0.5 → 0.8 → 1.0, first try each) |
| Best walk velocity tracking error | 0.028 m/s stochastic (gate: 0.030) at 0.02–0.06 m/s commands |
| Cross-leg current imbalance (hold) | 2.2 → ~1.37 (max-leg current / mean) |
| Tracking success at 0.08+ m/s commands | 0/6 — every widening attempt regressed |

## The five rounds, hypothesis → verdict

### Round 1
| Run | Hypothesis | Verdict |
|---|---|---|
| cw-stance-even | Per-servo hot-current penalty (`k_current_hot=0.2` above 1.5 A) + sparse foot-contact bonus (0.5) kills the learned 3-leg tripod stance | Partial. Sustained hot time halved (3.7 → 1.5–2.4 s) but tripod unchanged — the contact bonus is gradient-dead (a hovering foot earns nothing until it touches) |
| cw-stand-dr05 | Stand↔belly (rise+lower) survives DR 0.2→0.5 warm-started unchanged | PASS — rise 6/6 from all three start kinds, lower 6/6, det + stochastic |
| cw-walk2-gait | Swing touchdown bonus (`k_walk_swing=1.0`, one-shot per real swing: airborne ≥2 ticks, lands ≥15 mm away) breaks the "skating" gait | Partial. Stride 12→23 mm, slip −20%, forward distance 2×, but velocity tracking still 0/6 |
| cw-walk-fresh-gait | The balance-lineage prior is the local-optimum trap; fresh init + swing bonus escapes it | Refuted — fresh init converged to the identical skate. The trap is the objective landscape, not the prior |

### Round 2
| Run | Hypothesis | Verdict |
|---|---|---|
| cw-stance-clear | Dense stance-clearance penalty (`k_stance_clearance=10`, pays per mm a foot hovers above its episode-start height) supplies the missing gradient | Tripod BROKEN — all six feet load during hold. But raise collapsed to 0/6 (the penalty punished the lift itself) |
| cw-stand-dr08 | Ladder rung DR 0.5→0.8 | PASS — rise 6/6, lower 6/6 |
| cw-walk-slow | Commands (0.03–0.12 m/s) were mostly unreachable, so tracking never engaged; narrow to 0.02–0.06 m/s | WIN — vel err 0.064 → 0.036 m/s. First run where tracking visibly engages |
| cw-walk-prog3 | 3× progress reward (`k_walk_prog`) makes moving beat standing | Refuted — most gross motion of any run, zero tracking gain (0/6). Re-pricing without reachability does nothing |

### Round 3
| Run | Hypothesis | Verdict |
|---|---|---|
| cw-stance-raisefix | Exempt `raise` mode from the clearance penalty; raise recovers while stance stays even | PASS — raise 5/6 back, six-foot stance kept, rise/lower 6/6, imbalance ~1.37 |
| cw-stand-dr10 | Final rung: full DR 1.0 | PASS — rise 6/6 all starts, lower 6/6. **Hardware candidate** |
| cw-walk-slow2 | Consolidate at 0.02–0.06 before widening | GATE HIT — 5/6 stochastic at vel err 0.028 |
| cw-walk-curr08 | Widen straight to 0.02–0.08 without consolidating | Missed (0.043) — consolidate-then-widen confirmed as the pattern |

### Round 4
| Run | Hypothesis | Verdict |
|---|---|---|
| cw-stance-dr08 (even-stance line) | Even-stance policy up the DR ladder | PASS — hold 6/6 six-footed, raise 5/6, rise/lower 6/6 at DR 0.8 |
| cw-walk-w08 | Widen consolidated champion to 0.02–0.08 | Regressed — 1/6, vel err 0.041, and rise eroded to 2/6. Jump too big |
| cw-walk-w08-s1 | Seed-1 twin measures run variance | INVALID — bit-identical weights to its sibling after 5M steps. Root cause: SB3 `PPO.load` restores the ancestor's seed and `learn()` re-seeds torch+envs from it, silently ignoring `--seed` on every warm start. Fixed by overwriting `model.seed` after load. All previous warm-start "seed comparisons" were two copies of one run |
| cw-walk-dr04 | Walk skill survives DR 0.2→0.4 | Near-miss — 3/6 stochastic at 0.031 (gate 4/6 at ≤0.030) |

### Round 5 (in flight at time of writing, ~3 h in)
| Run | Experiment | Latest periodic eval |
|---|---|---|
| cw-stance-dr10 | Even-stance line, final DR 1.0 rung | rise 2/2 all starts, lower 2/2, hold 0.50° — on gate pace |
| cw-walk-w07 | Half-size widen 0.02–0.07 m/s, seed 0 | walk err 0.037 m/s |
| cw-walk-w07-s1 | True seed-1 twin (post seed fix) | walk err 0.030 m/s — seeds genuinely diverge now |
| cw-walk-dr04b | Consolidate walk at DR 0.4 (same settings, more steps) | walk err 0.037; flat-rise flapping 0/2↔2/2 |

## What's working

1. **Stand↔belly is solved and robust.** Rise from flat/bridge/crouch and
   lower, 6/6 det+stochastic at full DR 1.0. Two hardware-candidate
   champions: `cw-stand-dr10` (plain) and the even-stance line
   (`cw-stance-raisefix` → dr08 → dr10 pending).
2. **Dense-gradient reward design.** The clearance-per-mm penalty broke the
   tripod in one run where the sparse contact bonus did nothing. Same
   lesson as the earlier curl ratchet: if exploration can't feel reward
   before the behavior is complete, PPO never finds it.
3. **Speed curriculum for walking.** Reachable commands → tracking engages →
   consolidate at that range → widen in small steps. The only lever that
   ever improved velocity tracking. Reward re-pricing without reachability
   was cleanly refuted twice (prog3, and implicitly fresh-gait).
4. **The exact-path eval harness.** Duty cycles, per-servo currents, and
   gait metrics caught the tripod stance, the raise collapse, and the seed
   bug. None of these were visible in W&B scalar summaries.
5. **DR ladders warm-started rung-to-rung.** Every stand-line rung passed
   first try; cheap robustness once the skill exists at low DR.

## What's not working

1. **Walking beyond ~0.05 m/s.** Both range-widening attempts regressed.
   Current best is a deliberate 3–4 cm/s shuffle that tracks slow commands
   (stride ~23 mm). Open question: is this a curriculum-pace problem or
   does the gait itself (near-zero flight, high duty) cap achievable speed?
2. **Raise (10–30 mm body lift from stance)** hovers at 4–5/6 across every
   lineage; collapsed entirely under the clearance penalty until exempted.
   The small-lift reward appears marginal relative to competing terms.
3. **Rise erosion in walk runs.** The walk-heavy goal mix (~70%) crowds out
   rise practice; flat-rise flaps between 0/2 and 2/2 in walk-line evals.
   Likely needs mix rebalancing, interleaved rehearsal, or a later
   merge-by-distillation of the stand and walk champions.
4. **Peak currents.** All policies brush 2.5–2.7 A peaks (hardware breaker:
   2.5 A sustained). Sustained hot time improved a lot, but torque→current
   calibration hasn't been re-validated since the MuJoCo 3.11 upgrade
   shifted physics (quiet-hold peaks moved 2.46 → 2.60 A). Do not trust
   sim current absolutes for hardware gating yet.
5. **Walk observations are privileged.** Body-frame velocity comes from the
   simulator state. Deployment path (frame-stacking, recurrent policy, or
   teacher-student distillation) is designed but untested.

## Known good practices (hard-won, keep following)

- Warm-start + small deltas; one variable per run where possible.
- Gate on the stochastic harness eval, not W&B return curves.
- `--seed` on warm starts only works with the `model.seed` overwrite fix.
- Keep-best checkpoint policy: champions are archived per skill
  (`ppo_goal_<run>.zip`); "latest" is not "best".
- Consolidate before widening any curriculum axis.

## Open questions for the reviewer

1. How to widen the walk speed range without catastrophic forgetting —
   smaller rungs? mixed-range sampling (always include solved range)?
   separate walk specialist + later distillation?
2. Is the shuffle gait a dead end for >0.05 m/s? Would explicit gait-phase
   shaping (e.g. tripod-phase reward or foot-clearance targets during
   swing) help or fight the learned solution?
3. Best way to stop rise erosion during walk training — rehearsal mix,
   EWC-style regularization, or accept specialists + distill?
4. Raise is stuck at 4–5/6 everywhere. Worth a dedicated diagnosis run, or
   fold into the distillation plan?
5. Hardware next step: the stand↔belly champion meets its sim gates at DR
   1.0. What additional sim checks (current recalibration under MuJoCo
   3.11 above all) before a supervised on-robot trial of rise/lower only?
