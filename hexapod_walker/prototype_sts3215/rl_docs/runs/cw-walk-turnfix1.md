# cw-walk-turnfix1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-10T20:48:45+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-walk-yawgate2

**wandb_id**: ei9h0nkw

**hardware_ready**: False

**hypothesis**: Plain English: does a reward that can go NEGATIVE for turning the wrong way (instead of just paying less for turning the right way) finally stop the walking policy's fixed left-drift from swallowing every turn command? Every prior turn arm (yawcmd1, yawgate1, yawgate2 -- kernel-only, achieved-rotation gate, then a higher kernel price) failed because the reward SHAPE near zero-yaw was too flat to push against a small existing drift, and turning-in-place was <8% of training exposure. Today's landed fix is three parts used TOGETHER (signed rotation income reward.k_yaw_prog, a heading-hold drift charge reward.k_yaw_still, and a turn-in-place training curriculum goal.walk_turn_in_place_frac) -- all three passed a pre-training bank proving the combined stack prices honest turning above the drift. This is the FIRST time they train a real policy. DISCOVERY budget (2M steps, binary question) per RESEARCH_RULES. If-true: eval_yaw.py shows turn |wz_err| median dropping toward the 0.10 gate (from yawgate2's flat 0.236) in BOTH turn directions, with hold |wz| staying near the 0.05 gate -- worth a hardening budget. If-false: both directions still converge to the same drift despite the new mechanism -- turning needs the heavier structural fix (mirror-symmetry augmentation) held in reserve, not more reward tuning.

**gate**: eval_yaw.py per-scenario: turn-segment |wz_err| median vs the 0.10 rad/s gate, hold |wz| median vs 0.05, BOTH turn directions (not just the one aligned with the drift) -- run it on this checkpoint AND the frozen parent (yawgate2/joyjit_dr05_c1) under the identical scripted-command panel (matched-parent control) so the delta is attributable. Early video at first eval (1M); kill on the behavioral-impossibility rule if both signs still converge to the ~+0.09 rad/s drift. JOYSTICK GATE (0 falls) + forward-band retention must hold.

**verdict**: Matched-parent control (eval_yaw.py, identical scripted panel on turnfix1 vs frozen parent yawgate2): turn |wz_err| med 0.232 vs parent's 0.233, hold |wz| med 0.108 vs parent's 0.091 -- statistically the SAME as the already-failed parent, with the identical left/right asymmetry (arc-left ~0.07-0.21 near-drift-aligned, arc-right ~0.22-0.37 fighting the drift) that defines the structural left-yaw drift. The landed 3-part mechanism (signed rotation income k_yaw_prog, heading-hold drift charge k_yaw_still, turn-in-place curriculum) passed its pre-training bank but produced ZERO measurable behavior change in a real policy -- behavioral-impossibility kill per RESEARCH_RULES (yaw output stays command-invariant despite the reward separation existing on paper). Straight walk still clean (gv 6/6, 0 falls, normal 6-leg video) -- this is a turning-specific failure, not a training crash.

