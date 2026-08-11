# arch — Advanced architectures

W&B: tag `track:arch`. Excess-capacity research.

**Goal:** get a more advanced model (GRU/recurrent/temporal) to walk,
stand up, and sit down. What architecture learns the full skill set,
at what budget, with which failure modes.

## Now

- **From-scratch GRU walking CLOSED (08-11, gru-r4c):** both
  pre-registered levers (BPTT window 64→256 steps, hidden 128→256)
  tried and the identical leg-sacrifice/paddle fingerprint survived
  (legs [0,2,3] parked, det gait_valid 0/6) — worse, rise also
  regressed to 0/6 (r3 had it at champion grade). Reward already
  works on the MLP lineage, so this is a capacity/architecture limit,
  not a pricing gap. No further from-scratch GRU variant.
- Frame-stack line passed: hist16 → hist16-dep1 (deploy contract).
- **BC-distill-then-RL-finetune tried (08-11, `cw-arch-gru-bc-ft1`):
  walk survives, stance does NOT.** The GRU was BC-distilled from
  both specialist champions (walks + holds out of the box per the
  distill eval) then PPO-finetuned 10M steps on a walk-heavy mix
  (60/15/15/10). Result: walk stays honest (gait_valid 6/6, median
  prog_ratio ~0.85, no exploit) but rise collapsed to 0/6 det (honest
  sprawl/stall 20-108mm short on all three start kinds, not a cheat)
  and hold missed its tight stillness bar (track_err 2.43° vs 1.5°
  required; posture/height otherwise fine) — lower still lifts
  (4/6). Diagnosis: catastrophic forgetting of the imitation-learned
  stance under walk-heavy RL pressure, not a reward bug. Keeping
  `ppo_goal_cw_gru_bc.zip` (pre-finetune) as the reference artifact.

## Next

- Pre-registered next lever (not yet tried): SAME BC-distilled parent,
  smaller lr + tighter target-kl on the finetune (protect the
  imitation-learned stance from being overwritten) — NOT more steps,
  NOT a from-scratch variant. Queued as `cw-arch-gru-bc-ft2`.
- Later: contact-from-proprioception aux head; distill specialists
  into one recurrent net.

Detail: RL_PLAN.md "Architecture" · ledger cw-arch-* lineage.
