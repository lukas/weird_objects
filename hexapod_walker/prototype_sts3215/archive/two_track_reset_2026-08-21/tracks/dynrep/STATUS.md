# dynrep - dynamics representation pretraining

Last compacted: 2026-08-20 UTC. Excess-capacity research track. This file is
a current dashboard; detailed tournament history belongs in run docs, W&B,
`rl_docs/DYNREP.md`, and `RL_LOG.md`.

## Goal

Learn action-conditioned dynamics representations that make later PPO easier
or more reliable than the current scratch/BC-initialized policies.

## Current Sprint Rule

SIM SPRINT blocks new dynrep launches unless directly relevant to the
rise+walk download answer or explicitly ordered. The track currently has no
replacement for the download hierarchy.

## Current State

- The walk-curriculum tournament is terminal pending a new operator order. V1 stayed stuck at B0 for 40M; V2 was superseded; V3/V4/bridge/live-actor variants all failed their own gates.
- `cw-dynrep-tf-liveactor-walkcurr4-canary1-r1` failed the operator five-bar gate: held-out prediction worsened and zero guarded encoder snapshots were accepted. No 40M successor.
- `cw-dynrep-criticD-walkcurr4-bridge2` failed before actor release because critic EV never reached the readiness threshold. No bridge3 without a new hypothesis/order.
- `cw-dynrep-criticD-40m1` produced a real partial signal: a 6M-best checkpoint walked, but longer training harmed locomotion quality. This is evidence, not a deployment change.
- The best reading is not "representations never help"; it is that this stack has not beaten `bcgait1_hard1`, and value/critic competence remains the limiting layer.

## If Reopened

Require a simpler, pre-registered question before launch:

1. Prove the value head can learn useful returns on the exact rollout distribution.
2. Keep hard1 retention terms and matched-parent controls in the gate.
3. Do not run another bridge/walkcurr variant unless it isolates a new mechanism rather than re-spinning the tournament.

## Operator Gates

- Whether dynrep should receive any SIM SPRINT capacity.
- Whether to park the current walkcurr tournament, redesign the value-learning layer, or return to offline representation work later.

Keep under 120 lines. Replace stale bullets; do not append history.
