# arch — Advanced architectures

W&B: tag `track:arch`. Excess-capacity research.

**Goal:** get a more advanced model (GRU/recurrent/temporal) to walk,
stand up, and sit down. What architecture learns the full skill set,
at what budget, with which failure modes.

## Now

- GRU from scratch learns every STANCE skill to champion grade
  (gru-r3) but walks into the leg-sacrifice/paddle cheat.
- Frame-stack line passed: hist16 → hist16-dep1 (deploy contract).

## Next

- gru-r4 (10.24s window + 256 hidden) decides: if the cheat survives
  both levers, from-scratch GRU walking closes at this budget and
  recurrence waits for flagship distillation.
- Later: contact-from-proprioception aux head; distill specialists
  into one recurrent net.

Detail: RL_PLAN.md "Architecture" · ledger cw-arch-* lineage.
