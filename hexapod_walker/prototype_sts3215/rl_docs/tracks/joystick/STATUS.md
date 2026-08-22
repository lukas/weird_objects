# joystick - RL from the programmatic gait to joystick control

Last updated: 2026-08-22 (phasedir3 reprice launched). Keep this a
short screenful: Goal / Now / Next. Run detail lives in
`rl_docs/runs/`, W&B, and `RL_LOG.md`.

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
  trained on the convention-corrupted sim).
- 08-22 rung A verdict (`cw-dep-bcgait4-phasedir2-staged-fwd`,
  staged curriculum fb_20260822T032514, aligned-reward stack): FAIL
  on the pre-registered obedient-but-slow branch — zero falls,
  gait 6/6, slip 1.06x clone, dir_err -4.4deg BETTER, but progress
  0.836x clone (<0.9x) at the 0.060 band floor. Rung B NOT launched
  per gate. WIN inside the fail: the phase-locked BC anchor fully
  preserved the gait (first phasedir arm with zero behavioral
  damage). ROOT CAUSE: overspeed/loadslip charges are per-tick on
  the stochastic rollout, so exploration noise (std stuck 0.36,
  clone pays the same sto bill) pays them and the cheapest gradient
  is a slower mean gait. Charges must price stride-EMA/mean
  behavior, not tick noise, before any relaunch.
- 08-22 findings inherited from the reset window: the semantics-bank
  convention leak is repaired (`sim_gait_compat.py`) with a 7-test
  tibia-150 recalibration residue; the download hierarchy hard-fails
  the session gate at tibia-150. Fix arm `cw-dep-bcgait1-plant150-1`
  landed PASS(core) this cycle (0/6 falls DR-0+own-DR, session
  back-fall gone, fwd yaw -21.8->-10.6deg; promoted as the walk half).
  `cw-stand-footlow2-plant150-1` stays blocked: re-minting the rise
  reference is NOT a stale-file fix as first assumed — the scripted
  OPEN-LOOP belly->plant blend in `extract_rise_ref.py` (linear
  joint-angle interpolation from the champion's crouch-stand to
  `env._plant_deg`) now FALLS on every seed (0-24) at tibia-150, and a
  2x slower blend makes tracking error worse, not better (10.1 vs
  8.5deg), ruling out a timing fix. The longer tibia likely makes the
  linearly-interpolated intermediate pose invalid even though both
  endpoints are fine; needs an IK-anchored or foot-locked blend, not a
  re-run. This is probably the same root blocker under the stance fix.

## Next

1. **Finish the 7-test tibia-150 bank recalibration.** Blocker
   sharpened 08-22 (see above): fix `extract_rise_ref.py`'s
   belly->plant blend to use an IK/foot-anchored path instead of raw
   joint-angle lerp, re-mint `rise_ref_belly2plant.npz`, then
   re-price the 7 threshold tests (rise_valid_plant, rise_rock,
   trans_drag, getup_honest_ordering, recover_floor_rungs, fastprof)
   against the corrected reference. Green banks unblock
   `cw-stand-footlow2-plant150-1` and all aligned-reward work.
2. **Build the gate harness**: a 60 s randomized joystick session
   evaluator (held-out command scripts, falls / heading obedience /
   slip-per-m / per-leg gait metrics, video) + a
   `test_task_semantics.py` bank proving the training reward ranks
   gate-passing behavior above every known cheat (park, paddle-creep,
   overspeed attractor, sacrificed leg).
3. **DONE 08-22 — noise-taxed charges repriced, rung A relaunched**
   as `cw-dep-bcgait4-phasedir3-fwd-reprice` (RUNNING, train-1,
   fresh from the phase clone, same clone-relative gate). What
   landed: (a) the stride-EMA loaded-slip idea was implemented and
   REFUTED by measurement (tau sweep 0.08-0.4: the EMA kills the
   honest det scuff signal faster than the noise — jitter-slip is
   real physical slip in the same stride band), so the operator's
   other branch was taken: loadslip ok/max 2.2/4.0 -> 7.0/10.0,
   above the measured noisy-clone band 4.8-6.4 (det-band anti-skate
   now owned by the eval gate + BC anchor, recorded honestly in the
   bank); (b) NEW cfg `reward.walk_course_overspeed_along=1`
   (default-off in code) prices overspeed on the unbiased
   along-command projection instead of the sway-inflated |v_ema|;
   (c) course charge floored at 0.04 m/s smoothed speed.
   `test_phasedir_semantics.py` 24/24 incl. 4 new NOISY-REGIME tests
   at std 0.36; the operator's launch bar (noisy obey > noisy
   shrunken-gait) measured ~112 > ~95, sign-flipped from phasedir2.
   KNOWN HOLE (q_20260822T0640Z): noisy mean-overdrive out-earns the
   noisy clone ~121 vs ~112 and is behaviorally unpriceable at
   std 0.36; containment = phase anchor + gate items (c)/(e) — if
   phasedir3 det-overspeeds, it is this hole, not a mystery.
4. RL fine-tune from the phase clone (and a walk-champion arm as
   control) with the reward aligned to the gate metrics, resuming
   the staged heading curriculum; extend budget while reward and
   gate metrics rise together.
5. Widen the command distribution toward the full joystick envelope
   (speeds, yaw, strafe, stops) and DR-harden to own-DR zero-fall.

## Rules of the road

- Reward rising + gate metrics bad = realign reward with the gate
  and/or continue longer — never a one-line FAIL
  (`RUN_INTERPRETATION_RULES.md`).
- Do not park on operator input; assume-and-go with a recorded
  assumption. Physical-robot items are the only true waits.
