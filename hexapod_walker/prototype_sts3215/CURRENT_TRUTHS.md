# CURRENT TRUTHS - accepted facts and rulings

Last compacted: 2026-08-21 (two-track reset; folded the 08-22
plant-gate/bank findings). Accepted current facts, not narrative. If
history or old prose disagrees with this file, this file wins.

## Mission

Two goals, and only two, until both gates are green (operator,
08-21): (1) `joystick` — RL from the scripted programmatic gait to
joystick control, gated on 60 s of command-following in MuJoCo with
zero falls and teacher-band slip; (2) `amp` — the from-scratch AMP
program in `rl_docs/AMP_LOCOMOTION.md` (no Isaac Lab; MJX stack;
build all tools; done at M5 MuJoCo transfer). Operator-launched
out-of-scope runs get honest triage but no agent follow-ups.

## Current top rulings (operator, 08-21)

- Bad evals/canaries with training reward still rising is NOT a fail:
  continue the run longer and/or align the reward with the evals
  (`RUN_INTERPRETATION_RULES.md`). A known exploit on video means
  misalignment to repair, not a lineage kill.
- No operator pauses: assume-and-go with recorded assumptions. Only
  physical-robot access and spend approvals wait.
- While either gate is unmet, an idle fleet next to an empty backlog
  is the failure state; build tools, fund continuations, queue the
  next milestone arm.
- SIM SPRINT and the seven-track structure are superseded.

## Facts that feed the two tracks

- The scripted tripod teacher is verified clean at the measured
  tibia-150 plant (commit a4beb8af): 0.06-0.10 m/s x 4 headings, zero
  falls, slip/m 1.4-2.9, full fast servo profile. It is the joystick
  starting point and the amp motion-prior seed.
- The phase-conditioned BC clone
  `ppo_goal_cw_bcgait_init_fullprof_phase1` (holdout act err 0.0040)
  passes the entire direction-first curriculum with ZERO RL: all
  fixed headings incl. rear, irregular heading changes, stops.
- Five fast-gait RL levers failed AS RUN (faster cadence,
  k_walk_cmd_track, speed-obs+charges, +4M steps,
  phase-obs+fixed-speed). Under the 08-21 ruling these are
  reward<->eval alignment failures, not proof RL cannot improve the
  clone. CAVEAT (08-22): the 5th verdict (`cw-dep-bcgait4-phasedir1`)
  is ENV-CONFOUNDED — it trained on the convention-corrupted sim
  (30660b51) — so it needs a re-run on the repaired sim before any
  phase-RL conclusion; the phase INPUT itself kept zero falls, and
  the operator's staged heading curriculum (fb_20260822T003132:
  forward-only -> small heading set -> full headings -> irregular) is
  UNTRIED.
- BANK BREAK ROOT-CAUSED + REPAIRED (08-22): 30660b51's
  absolute-tibia knee convention + new default stand home leaked into
  the femur-relative sim (full semantics bank 43 FAIL). Fixed at the
  boundary by `linux_control/sim_gait_compat.py` + a
  `_default_plant_deg` guard; hardware untouched. Residue: 7 FAILs,
  all true tibia-150 recalibration (rise ref re-mint first) —
  agent-doable, and prerequisite for aligned-reward bank work.
- MEASURED-PLANT GATE BREAK (08-22): the download hierarchy
  HARD-FAILS the interactive session gate at tibia-150 (sit
  tilt_pitch + reverse tilt_roll falls, fwd yaw -21.8 deg) while the
  matched 128 mm control PASSES — the plant correction alone breaks
  the shipped answer; DOWNLOAD_ANSWER's n=600 numbers are old-plant
  facts. Fix arms: `cw-dep-bcgait1-plant150-1` (launched),
  `cw-stand-footlow2-plant150-1` (blocked on the rise-family bank
  residue). Evidence: `logs/ckpt_eval/plantgate_tibia150_session/`.
- direction_err_mean_deg has a ~35 deg tick-level floor from stride
  sway — judge deltas vs a matched clone, not raw values.
- Every pre-08-22 checkpoint (incl. the download hierarchy) trained
  on the old 128 mm plant; cross-plant comparisons need matched
  controls.
- Fallback deployable answer: `footlow2_hard1` stance +
  `bcgait1_hard1` walk + session controller (det 0.967, sto 0.853,
  n=600 — old-plant numbers; see the gate break above;
  `rl_docs/DOWNLOAD_ANSWER.md`).

## Real robot facts

- The robot is off the bench; all physical items are operator-owned.
  No physical motion without an explicit operator ask, ever.
- Scripted tripod gait walks, crabs, and turns on hardware with
  visible loaded-foot slip and roughly half commanded travel.
- Servo/control facts: 25 Hz loop, 1.5 deg/tick stateful slew, loaded
  settle ~250-325 ms, air settle ~9 ms. Working gaits rock 10-20 deg.
- Robot-control/web edits use `linux_control/dev_loop.sh` helpers
  (`make robot-check` / `robot-unit-check` / `robot-status` /
  `robot-deploy`); these never move the robot.

## Policy and eval facts

- Policies output 18 raw joint targets through the SafetyLayer.
- Video/physical behavior outranks reward alone; a reward-passing
  cheat is a metric bug, not a skill.
- Matched-parent controls are mandatory for injected physics/sensor
  axes.
- MJX/Warp GPU stack: 12 single-H200 train pods, ~4096 envs each,
  per-world model DR, canary probes, eval/video logging, desync.
