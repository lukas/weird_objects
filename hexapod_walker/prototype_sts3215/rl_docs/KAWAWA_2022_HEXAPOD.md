# Kawawa-Beaudan 2022 Hexapod — walkcurr track baseline

Source: Maxime Kawawa-Beaudan, Avideh Zakhor, *Learning to Walk:
Legged Hexapod Locomotion from Simulation to the Real World*, UC
Berkeley EECS technical report UCB/EECS-2022-146, May 19 2022.

Links:
- Paper page: https://www2.eecs.berkeley.edu/Pubs/TechRpts/2022/EECS-2022-146.html
- Project page: https://sites.google.com/view/hexapod-rl
- Public code link advertised by the project page (404 when checked
  2026-08-23): https://github.com/maximejkb/hexapod-isaac.git

## What they did

- Trained hexapod locomotion entirely in simulation, then transferred.
- Compared motion-prior policies against prior-free policies; the
  strongest flat/rough policies were PRIOR-FREE (the with-prior flat
  policy learned rapid foot-tapping that transferred poorly).
- Prior-free Isaac Gym policies output target joint positions
  directly; one-layer LSTM encoder (hidden 64), actor/critic MLPs
  128/64/32, ELU, PPO, 4000 parallel agents, 24-step rollouts.
- Proprioceptive state (attitude, joint positions, previous actions),
  no gait phase for prior-free policies.
- DR: friction 0.5-1.25, random external pushes every 8 s.

## Local adaptation — history

**First attempt `cw-kawawa2022-pf-flat1` (FAIL, 08-23,
rl_docs/runs/cw-kawawa2022-pf-flat1.md):** launched from an unpushed
desktop clone (temp commit b126ceb3; its RecurrentPPO/LSTM trainer
support is LOST — pod deploy copy overwritten, nothing reachable in
git). Trained on the full multi-goal joint_walk diet; the
hold/raise/track/unload sub-goals carried aggregate reward while walk
tilt-terminated at every eval from 8M to 40M. Post-hoc bank
measurement additionally showed the launch reward stack was misaligned
for the walk goal ALONE: parking out-earned clean walking +387 vs
+325 (WALKCURR_PF bank, test_task_semantics.py).

**Canonical re-registration (walkcurr track, operator order
20260823T154657Z):** recipe module `rl_move.sim.kawawa2022_recipe`,
first run `cw-walkcurr-pf-fwd1`. Changes designed out of the failure:

- `goal.walk_pure=1` walk-only diet (no other goal exists to carry
  reward);
- fixed forward command 0.05-0.06 m/s, heading 0, no resampling
  (curriculum rungs widen ONLY after a certified pass);
- bank-calibrated v2e pricing (freeprog 3.0 on stride-EMA velocity,
  step_event 1.0, park_duty 4.0, idle 2.0, heading 0.5, loadslip
  excess 4.5, term_penalty 1200) — measured ranking: gait +346 >
  stall -31 > park -352 > sideways -609 > reverse -741 > skate -1058
  > topple -1164 (the operator-required ordering, exactly);
- memoryless MLP 128/64/32 + ELU + 24-step rollouts for rung 1 (the
  paper's LSTM path died with the desktop clone; the in-repo
  recurrent option for later rungs is `--gru --gru-hidden-size 64`);
  body-velocity obs KEPT while memoryless (the paper's
  proprioception-only obs assumed the LSTM could estimate velocity);
- rung 1 is DR0 (paper's friction/push DR is rung-5 hardening).

## Difference from other tracks

- vs `cpg`: no low-dimensional controller search; PPO learns all 18
  joint targets.
- vs `joystick`: no tripod teacher, no gait clock, no BC anchor.
- vs `amp`: no discriminator or motion library; also NOT the harsh
  SLIPWALK pricing that statued 8 amp from-scratch arms.
