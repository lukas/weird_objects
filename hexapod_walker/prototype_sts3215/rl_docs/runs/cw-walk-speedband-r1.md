# cw-walk-speedband-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T13:30:40+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_anchorgate.zip

**wandb_id**: 1fgasaqi

**hardware_ready**: no

**hypothesis**: OPERATOR WISHLIST 8b (operator-tunable speed), REVISED after cw-walk-fast (cycle 42) refuted the fast end: the gait tops out ~0.065 m/s regardless of command, so the original 0.02-0.12 band was half-unreachable and its 0.11 gate leg failed by construction. One policy tracking the ACHIEVABLE 0.02-0.06 m/s band with mid-episode command resampling, so speed becomes a runtime knob within the gait's real envelope. If-true: tracking err flat across 0.03/0.045/0.06 with gait_valid and no parking at the slow end. If-false: slow commands collapse to parking or one head cannot serve the band - bucket into separate policies. Strongest alternative: mid-episode resampling itself destabilizes the gait (transition stalls).

**gate**: DR0 det+sto 6/6 at commanded 0.03/0.045/0.06 m/s incl mid-episode resample: gait_valid, zero terminations, per-speed tracking err within 25% of champion at 0.05; no park episodes at 0.03

**verdict**: FAIL (near-miss). Achievable-band (0.02-0.06) speed knob with 5s resample: no parking at the slow end, prog med ~1.1, gv 12/12, 0 term — speed IS commandable coarsely. But per-ep tracking err 0.029-0.042 with 4/12 eps >25% over champion's 0.029, and transition slip/m med 2.2 (2x champion). Same lesson as wander: command changes double slip. Precision speed tracking joins the contact-pricing queue; no requeue.

