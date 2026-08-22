# multitask - command-conditioned generalists

Last compacted: 2026-08-20 UTC. Excess-capacity track. The 08-13 pause was
lifted on 08-15, but SIM SPRINT still gates new launches unless they directly
serve the download answer or the operator explicitly orders them.

## Goal

Test whether a fresh command-conditioned generalist can learn stand, walk,
stop, yaw/lateral commands, and later transfer better than sequential
specialists.

## Current State

- `cw-joystick-translate1` failed: survival improved, but real along-command motion stayed near zero; video showed a parked/stilt-leg exploit.
- `cw-joystick-translate-scratch1` failed differently: clean six-leg cycling but march-in-place/no net displacement. Together, these close the exact joystick-translate reward/command recipe.
- Cross-track arch repeats reproduced the same problem: architecture changes and harsher fall pricing did not create a surviving, command-following gait.
- `cw-mt-b1-dualgru1` failed with no useful benefit; dual-core recurrence did not fix the narrow-generalist acquisition shortfall.
- The old yaw clauses for b2/c2 are invalid because yaw was commanded but not paid; future yaw claims need audited yaw reward and command metrics.

## If Reopened

Do not relaunch the same recipe. A valid next arm needs a changed task/reward
contract that pays actual along-command displacement and rejects march-in-place
or stilt/park behavior before training.

Required before launch:

1. MDP/preflight bank where real commanded translation beats park, stilt, march-in-place, and wrong-way motion.
2. Explicit audited yaw pricing for any yaw claim.
3. Plain physical headline metrics, not mechanism names.

## Operator Gates

- Whether multitask is worth capacity during SIM SPRINT.
- Whether the next attempt is reward/metric redesign, command curriculum, or parked until the specialist hierarchy is bench-promoted.

Keep under 120 lines. Replace stale bullets; do not append history.
