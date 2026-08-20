# cw-dep-bcgait1-midthru1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-20T19:34:34+00:00

**pod**: hexapod-mjx-train-4

**steps**: 1000000

**parent**: cw-dep-bcgait1-hard1

**wandb_id**: 6l0rxihi

**hypothesis**: Let PPO repair the step-0 wobble instead of aborting on it: this canary trains the proven tall walker under the mid 750/40/3-deg servo profile with the fast anti-skate curriculum (V5) and the direct loaded-slip penalty, with the fail-closed pre-PPO B0 cert WAIVED (operator order q_20260820T0830Z answer, MCP operator lane 20260820T191113Z: execute option (a) train-through - do not fail-closed solely on step-0 B0 wobble). The step-0 state is already on record (midnoslip1: falls 2/8, slip 1.66/m, roll 10.6 deg, 1.26x overshoot); the question is whether 1M steps of PPO at the target dose repairs B0 (falls to zero, command tracking back in band) or the wobble collapses training. Health/fall/slip aborts stay armed: periodic B0 cert every 500k with fail-streak rollback, fixed-seed canary auto-stop, 25-deg tilt + walk-height safety terminations. Companion arm cw-dep-bcgait1-midramp1 (same dose, profile ramp-in, cert kept) separates train-through from ramp-in causality; full-dose siblings fastthru1/fastramp1 read the dose axis.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature fast gait here. Pre-registered (operator pick (a), q_20260820T0830Z): (1) no pre-PPO abort by design - the init cert is intentionally waived per operator order; step-0 B0 numbers are already banked from cw-dep-bcgait1-midnoslip1, so improvement is judged against them; (2) by 1M, periodic B0 cert rounds (every 500k, det n=8, at the full mid 750/40/3-deg target dose) show falls trending to ZERO and cmd_prog_frac>=0.50, slip_per_m trending <=1.6, cross_track<=0.20, height_factor>=0.80 - PASS requires the final cert round at zero falls with those bars; (3) health: finite losses, no KL-rollback storm, no canary auto-stop, reward_loadslip_excess trending toward zero. FAIL if behavior collapses instead of repairing: falls not improving across two consecutive cert rounds, steer6-style skating (slip/m>2.5, direction/cross-track failure), tilt-exit storm, or canary auto-stop fires - kill early on any of these, do not ride out the budget. Strict post-training eval at the full target dose. Compare against cw-dep-bcgait1-midramp1 (same dose, option (b)) for clean A/B causality. DOWNLOAD_ANSWER and the hierarchy baseline are untouched unless this arm passes its gate.

