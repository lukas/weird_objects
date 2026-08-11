# Track: turn — Commanded turning

W&B filter: tag `track:turn` in l2k2/hexapod-balance.

## Goal (operator, 08-11)

Make commanded yaw actually work: fix the structural left-yaw drift
baked into the asymmetric walk gait so the joystick can point the
robot. De-scoped from the hw deliverable (no camera = no front, and
rot-60 covers translation headings), but alive as its own research
line on excess capacity.

## Current state (update when a verdict changes the story)

- 08-10/11: yawcmd/yawgate1/yawgate2 FAILED (fixed left-yaw drift);
  the signed-income/drift-charge/turn-curriculum set passed its TURN
  bank but cw-walk-turnfix1's matched-parent control was
  statistically identical to the failed parent — reward-shape tuning
  on this task is DOUBLY CLOSED. Root-cause reading: the drift lives
  in the gait itself, not the turn reward.
- The one untried lever: mirror-symmetry augmentation [CODE — trainer
  surgery]. Design, bank numbers, sign audit: rl_docs/TURN.md
  (the detail doc this track file fronts for).

## What belongs here

Mirror-symmetry trainer work, yaw-command arms and their
matched-parent controls, eval_yaw.py measurements, sim-vs-hardware wz
sign audits. Detail: rl_docs/TURN.md (design + failure history).
