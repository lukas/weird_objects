# cw-dep-bcgait1-fastthru1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-20T19:17:35+00:00

**pod**: hexapod-mjx-train-3

**steps**: 1000000

**parent**: cw-dep-bcgait1-hard1

**wandb_id**: 4azy8vad

**hardware_ready**: False

**hypothesis**: Let PPO repair the step-0 wobble instead of aborting on it: this canary trains the proven tall walker under the full 1500/80/5-deg servo profile with the fast anti-skate curriculum (V5) and the direct loaded-slip penalty, with the fail-closed pre-PPO B0 cert WAIVED (operator order q_20260820T0830Z answer, MCP operator lane 20260820T191113Z: execute option (a) train-through - do not fail-closed solely on step-0 B0 wobble). The step-0 state is already on record (fastnoslip1: falls 6/8, slip 2.26/m, roll 10.2 deg, 2.09x overshoot); the question is whether 1M steps of PPO at the target dose repairs B0 (falls to zero, command tracking back in band) or the wobble collapses training. Health/fall/slip aborts stay armed: periodic B0 cert every 500k with fail-streak rollback, fixed-seed canary auto-stop, 25-deg tilt + walk-height safety terminations. Companion arm cw-dep-bcgait1-fastramp1 (same dose, profile ramp-in, cert kept) separates train-through from ramp-in causality; mid-dose siblings midthru1/midramp1 read the dose axis.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature fast gait here. Pre-registered (operator pick (a), q_20260820T0830Z): (1) no pre-PPO abort by design - the init cert is intentionally waived per operator order; step-0 B0 numbers are already banked from cw-dep-bcgait1-fastnoslip1, so improvement is judged against them; (2) by 1M, periodic B0 cert rounds (every 500k, det n=8, at the full full 1500/80/5-deg target dose) show falls trending to ZERO and cmd_prog_frac>=0.50, slip_per_m trending <=1.6, cross_track<=0.20, height_factor>=0.80 - PASS requires the final cert round at zero falls with those bars; (3) health: finite losses, no KL-rollback storm, no canary auto-stop, reward_loadslip_excess trending toward zero. FAIL if behavior collapses instead of repairing: falls not improving across two consecutive cert rounds, steer6-style skating (slip/m>2.5, direction/cross-track failure), tilt-exit storm, or canary auto-stop fires - kill early on any of these, do not ride out the budget. Strict post-training eval at the full target dose. Compare against cw-dep-bcgait1-fastramp1 (same dose, option (b)) for clean A/B causality. DOWNLOAD_ANSWER and the hierarchy baseline are untouched unless this arm passes its gate.

**verdict**: CANARY FAIL - MECHANISM: train-through (option a) does NOT repair the full 1500/80/5 dose step-0 wobble; PPO drives it into total collapse instead. Both 500k cert rounds FAIL identically (no_falls,cross_track,slip,roll,duration; survive=2.0s both r1 and r2 - zero improvement across two consecutive rounds, the pre-registered kill trigger). Final gate eval at 1.05M: det 0/6 and sto 0/6 ok, ALL 12 episodes TERM walk_low_height, slip 4.4-9.6/m det (5.5 med), 3.0-36/m sto (9.4 med) - both far past the 1.6 target and the 2.5 skating disqualifier. Own-DR pass confirms: 0/6 det, 0/6 sto, slip 6.9/11.6 per m, roll_settled 0/6, dir_err ~60deg. Contact sheet shows progressive leg splay into a flattened collapse across the reel, not a repaired gait. Companion cw-dep-bcgait1-fastramp1 (same dose, ramp-in) is the causal control still training; do not judge ramp-in from this arm.

