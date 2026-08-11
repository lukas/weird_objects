# Track: hw — Hardware joystick robot

W&B filter: tag `track:hw` in l2k2/hexapod-balance.

## Goal (operator, 08-11)

A walking, joystick-driven, standing/sitting/holding robot working ON
HARDWARE **by any means necessary**. BC anchors, scripted blends,
rot-60 wrappers, specialist checkpoints, hand-tuned handoffs — all
fair game here; purity questions belong to the nobc track. The KPI is
unresolved blockers between the current robot and reliable joystick
control, session after session. This is the MAINLINE track: it has
priority for pods and for the operator's bench time; the other tracks
run on excess capacity.

## Current state (update when a verdict changes the story)

- 08-11: Sim side is largely SOLVED: rise + quiet hold
  (`ppo_goal_cw_stand_holdbc1_hard1`), walk champion
  (`cw-dep-vref1-r1`, THE attempt-#2 checkpoint), full-circle
  translation via rot-60, full motion cycle composes with zero falls
  (rise -> walk -> stop -> sit). Deploy ports for the stance
  specialist and rot60 are LANDED; remaining work is BENCH-ONLY
  (hardware attempt #2, operator-supervised).
- Open on this track: hardware attempt #2 (the true critical path),
  loaded/contact/current calibration (open problems 1+4), walk-height
  command (unsolved), flagship unified-checkpoint question (MoE fork
  pending after cw-uni-flag-a1-h2), crouch-start rise fragility
  (state-aligned BC anchor is next, CODE/spec first), BC-anchor gait
  cleanup (`rl_docs/GAIT.md` P1/P2 — anti-scrape for hardware).

## What belongs here

Deployment-contract arms (Gate 0), hardening/composition on deploy
candidates, handoff/composition evals, runner/deploy code, bench
sessions, GAIT.md P1 (BC gait cleanup) + P2 (structural terms as a
deploy tool). Detail docs: RL_PLAN.md queue, rl_docs/HARDWARE.md,
RISE.md, GAIT.md, SKILLS.md.
