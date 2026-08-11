# Track: arch — Advanced architectures

W&B filter: tag `track:arch` in l2k2/hexapod-balance.

## Goal (operator, 08-11)

Get a more advanced model — GRU/recurrent/temporal — to walk, stand
up, and sit down. This is capability research: what architecture
learns the full skill set, at what budget, with which failure modes.
Not on the hardware critical path; runs on excess capacity.

## Current state (update when a verdict changes the story)

- 08-11: GRU from-scratch learns every STANCE skill to champion grade
  (cw-arch-gru-r3) but walks straight into the leg-sacrifice/paddle
  cheat; r4 (10.24s window + 256 hidden) is the window+capacity
  test — if the fingerprint survives both levers, from-scratch GRU
  walking closes at this budget and recurrence waits for flagship
  distillation. Temporal frame-stack line: hist16 passed and produced
  hist16-dep1 (deployed-contract variant).
- Open questions: GRU walk cheat (r4 pending), history-length
  saturation, contact-from-proprioception aux head (queued),
  distillation of specialists into one recurrent net.

## What belongs here

Architecture variants (GRU, history stacks, aux heads, MoE
experiments when the flagship forks), their matched-parent controls,
and architecture-specific infrastructure (BPTT window, hidden sizing,
iteration-count parity). Detail: RL_PLAN.md "Architecture", TURN.md
for mirror-loss machinery shared with the turn track.
