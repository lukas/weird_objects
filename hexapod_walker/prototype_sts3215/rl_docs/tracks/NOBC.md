# Track: nobc — Learn without BC anchor

W&B filter: tag `track:nobc` in l2k2/hexapod-balance.

## Goal (operator, 08-11)

Learn standing from scratch WITHOUT the help of a BC anchor — and
more broadly, honest skill discovery by RL alone: structural rewards,
curricula, terrain, RSI, physics easing. No imitation losses of any
kind. The operator's standing objection: "I'm disappointed that we do
everything with a BC-anchor; it should be possible without." The hw
track ships whatever works; this track exists to retire the crutch.

## Current state (update when a verdict changes the story)

- 08-11: Stand-from-scratch history: six reward-side arms collapsed
  identically (score stack, ref-track, RSI — RISE.md forensic
  ladder); the collapse was later attributed to warm-start OOD drift,
  and the Warp pool-restore bug contaminated the early verdicts, so
  income-shaping/RSI verdicts are formally REOPENED for from-scratch
  designs. RSI machinery works and is validated.
- Gait-from-scratch (GAIT.md P3): terrain clamp bug found+fixed
  (every historical terrain run was really <=18mm bumps);
  cw-gait-terrain2 (true 72mm, physics-as-teacher) FAILED into
  leg-sacrifice; cw-gait-dragstep1 (4x drag charge) FAILED into
  paddle — with the decisive caveat that the effective charge never
  exceeded ~0.09/tick vs ~1/tick income. The "really big penalty"
  hypothesis is UNTESTED, not refuted.
- Next arms (GAIT.md): drag-charge MAGNITUDE AUDIT, then one
  from-scratch arm with the charge sized so a paddle tick costs 2-3x
  a progress tick; RSI-for-walk mid-stride spawns; annealed-up charge
  once the P2 bank lands; stand-from-scratch resumes with the same
  toolbox once gait answers which levers move discovery at all.

## What belongs here

From-scratch discovery arms with structural rewards/curricula/RSI/
terrain, their banks and probes, and the charge-magnitude audit. NO
imitation losses — an arm that needs one belongs to hw. Detail:
GAIT.md P3, RISE.md.
