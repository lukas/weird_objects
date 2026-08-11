# cw-walk-gaitbc1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-11T17:11:10+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-dep-vref1-r1

**wandb_id**: 6f0ltn7m

**hardware_ready**: False

**hypothesis**: GAIT CLEANUP P1 (operator 08-11, RL_PLAN queue -0.5, rl_docs/GAIT.md): the BC anchor can break the PADDLING HABIT on a walker that already travels. Every champion drags loaded feet (slip/m 1.1-1.5) even though pricing prefers stepping (probe_walk_income: honest gait out-earns the paddle 2-4x) and even though k_drag_loaded=10 is ALREADY in this stack -- the paddle is a strong local optimum PPO finds first, the same entrenched-habit class the anchor broke twice (rise cw-stand-bc1, hold cw-stand-holdbc1). Unlike cw-omni-transbc1 (from-scratch omni TRANSLATION -- a geometry problem rot-60 solved), this arm warm-starts THE hardware walker cw-dep-vref1-r1 so the SKILL (traveling, contract obs, tilt envelope) is already there and the anchor only has to reshape HOW it travels toward the scripted lift-and-place tripod (tape-proven on the real robot). ONE variable vs the parent recipe: train.bc_anchor_coef=1.0 (+ --init-from the parent champion).

**gate**: slip/m at DR0 AND own-DR0.35 must drop decisively below the parent band: <0.6 vs parent 1.1-1.5, at matched travel (fwd distance and vel-err within the parent's joystick-gate band, zero falls, gait_valid). Kill signature (pre-registered): fwd distance collapses toward the scripted gait's band with no slip win, or park-and-earn returns, or bc_anchor_loss converges while slip stays >1.0 (anchor satisfied by near-miss timing without lift -- then P2's structural swing-clearance term is the next lever, not a coef variant).

**verdict**: STOP -- known exploit: FREEZE. GAIT CLEANUP P1 (BC-anchor on walk ticks) collapsed the hardware walker into a static frozen tripod-like pose (3 legs held aloft motionless, 3 planted, IDENTICAL across all 6 det episodes and both DR0/own-DR0.35 -- video-confirmed) instead of a lift-and-place gait: fwd travel ~0.00m/15s (parent band 0.28-0.34m), gait_valid 1/6 det DR0 (2/6 at own-DR), 5/6 legs flagged 'sacrificed' every episode. Matches the pre-registered kill signature ('anchor... without lift' / fwd collapse) at its worst extreme -- not partial, total freeze. Training-time diagnostic already flagged this (GAIT.md): bc_anchor_loss converged cleanly (5.16->0.03) while walk_loadslip_factor collapsed 1.0->0.06, i.e. the anchor was satisfied by NOT MOVING, not by lifting feet. Unlike rise/hold (stationary skills the anchor fixed twice), walk's anchor target is itself in motion every tick; combined with the existing income-gated walk reward, the two objectives found a degenerate joint optimum (freeze) rather than reshaping the gait. Joystick-gate ONLY (eval_drive, zero falls) is not informative here since it doesn't check travel magnitude the way the harness does; discard as not evidence of health. Per the run's own pre-registered gate text, this is NOT a coefficient-variant situation -- move to P2 (structural stance-slip charge + swing-clearance term, bank first) or P3 lever 4 (RSI-for-walk), not another bc_anchor_coef dose on this application.

