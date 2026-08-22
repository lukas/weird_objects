# arch - advanced architectures

Last compacted: 2026-08-20 UTC. Excess-capacity track. This file is a
current dashboard, not a chronology. Details live in run docs, W&B, and
`MODE_EXPERTS_DIRECTIVE.md`.

## Goal

Find whether GRU/recurrent/transformer/mode-expert architectures can learn
the full stand/walk/sit skill set better than the current specialist
hierarchy.

## Current Sprint Rule

SIM SPRINT blocks new arch launches unless the arm directly improves the
rise+walk download answer or the operator explicitly orders it. Nothing in
this track currently changes `rl_docs/DOWNLOAD_ANSWER.md`.

## Current State

- Mode-experts scratch path: `cw-arch-modeexperts-scratch2` showed genuine learning but no skill success; `scratch3` regressed into the known 3-planted/3-frozen exploit in rise/hold/lower while walk stayed good. No `scratch4`.
- Joystick transformer path: `cw-arch-joystick-canary1` failed its canary. Critic EV stayed near zero because episodes collapsed at the low-height grace boundary; no 40M clone.
- DAgger/BC mode-expert path: `bc2` improved isolated rise but made the sequence worse, especially post-lower rise. That closes the current DAgger recipe ladder.
- From-scratch GRU walking is closed at the tested budgets/mechanisms. Bigger memory/routing did not fix walk acquisition.
- Causal/transformer work produced useful diagnostics and partial walking signals, but no deployable replacement for the stance/walk hierarchy.

## If Reopened

Do specification before training:

1. Add rise/hold/lower anti-flag/anti-tripod preflight banks equivalent to the walk anti-park checks.
2. For joystick canaries, inspect initial pose/height reference versus the 25 mm low-height trip band before any new run.
3. Treat any new architecture arm as research unless its gate explicitly beats the current download hierarchy.

## Operator Gates

- Whether to spend any more SIM SPRINT capacity on arch at all.
- Whether the next arch attempt is anti-cheat task-spec work, value-learning repair, or parked until after hardware promotion.

Keep under 120 lines. Replace stale bullets; do not append history.
