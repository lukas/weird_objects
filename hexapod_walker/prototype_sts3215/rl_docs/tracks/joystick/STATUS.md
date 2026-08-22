# joystick - RL from the programmatic gait to joystick control

Last updated: 2026-08-21 (track created by the operator two-track
reset). Keep this a short screenful: Goal / Now / Next. Run detail
lives in `rl_docs/runs/`, W&B, and `RL_LOG.md`.

## Goal

Start from the simple programmatic gait (the scripted tripod teacher
and its BC clones) and use RL to make it genuinely joystick
controllable in sim.

**DONE gate (pre-registered, operator 08-21):** one policy (or the
session-controller stack) follows a randomized 60-second joystick
command script in MuJoCo — direction changes, stops, reverses, turns —
with:

- ZERO falls across the full panel (n>=12 episodes, det+sto, DR-0 and
  the run's own DR, held-out command seeds);
- directions actually followed (heading obedience judged against the
  teacher clone's measured ~35 deg tick-level stride-sway floor —
  compare deltas, not raw values);
- little slip: slip/m no worse than the scripted teacher's measured
  band at the calibrated plant (<= ~2.9; teacher band 1.4-2.9).

## Now (inherited state, 08-21)

- Scripted tripod teacher verified clean at the measured tibia-150
  plant: 0.06-0.10 m/s x 4 headings, zero falls, slip/m 1.4-2.9,
  full fast servo profile.
- Best starting checkpoint: the phase-conditioned BC clone
  `ppo_goal_cw_bcgait_init_fullprof_phase1` (holdout act err 0.0040)
  passes the entire direction-first curriculum with ZERO RL — all
  fixed headings incl. rear, irregular heading changes, stops.
- Fallback baseline: the download hierarchy
  (`footlow2_hard1` stance + `bcgait1_hard1` walk + session
  controller; held-out session gate det 0.967 / sto 0.853, n=600).
  Note: pre-08-22 checkpoints trained on the old 128 mm plant.
- Hard-won evidence: five fast-gait RL levers failed AS RUN because
  the reward was not aligned with the eval (faster cadence, tracking
  price, speed-obs+charges, more steps, phase-obs+fixed-speed).
  Per the 08-21 interpretation ruling those are MISALIGNMENT results,
  not dead ends: RL on this track requires a reward whose optimum is
  the 60 s gate behavior, then enough budget. The 5th verdict
  (`cw-dep-bcgait4-phasedir1`) is additionally ENV-CONFOUNDED (it
  trained on the convention-corrupted sim) — re-run on the repaired
  sim; the operator's staged heading curriculum
  (fb_20260822T003132) is untried.
- 08-22 findings inherited from the reset window: the semantics-bank
  convention leak is repaired (`sim_gait_compat.py`) with a 7-test
  tibia-150 recalibration residue (rise ref re-mint first); the
  download hierarchy hard-fails the session gate at tibia-150 (fix
  arms: `cw-dep-bcgait1-plant150-1` launched,
  `cw-stand-footlow2-plant150-1` blocked on the rise-family banks).

## Next

1. **Finish the 7-test tibia-150 bank recalibration** (rise-ref
   re-mint first) — green banks are the prerequisite for all
   aligned-reward work and unblock the stance fix arm; triage the
   in-flight `cw-dep-bcgait1-plant150-1` when it lands.
2. **Build the gate harness**: a 60 s randomized joystick session
   evaluator (held-out command scripts, falls / heading obedience /
   slip-per-m / per-leg gait metrics, video) + a
   `test_task_semantics.py` bank proving the training reward ranks
   gate-passing behavior above every known cheat (park, paddle-creep,
   overspeed attractor, sacrificed leg).
3. RL fine-tune from the phase clone (and a walk-champion arm as
   control) with the reward aligned to the gate metrics, starting
   from the staged heading curriculum on the repaired sim; extend
   budget while reward and gate metrics rise together.
4. Widen the command distribution toward the full joystick envelope
   (speeds, yaw, strafe, stops) and DR-harden to own-DR zero-fall.

## Rules of the road

- Reward rising + gate metrics bad = realign reward with the gate
  and/or continue longer — never a one-line FAIL
  (`RUN_INTERPRETATION_RULES.md`).
- Do not park on operator input; assume-and-go with a recorded
  assumption. Physical-robot items are the only true waits.
