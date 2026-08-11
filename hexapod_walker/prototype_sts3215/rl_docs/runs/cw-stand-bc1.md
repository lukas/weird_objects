# cw-stand-bc1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-11T04:06:55+00:00

**pod**: hexapod-mjx-train-6

**steps**: 2000000

**parent**: cw-stand-rsi3

**wandb_id**: bhvbnu7s

**hypothesis**: Teach the robot to stand up by holding its hand along the known-good path: a supervised anchor in the trainer now pulls the policy's actions toward the recorded stand-up motion at every step of a rise episode, so the warm-started skill cannot drift away into the three-legs-in-the-air cheat while learning. This arm tests whether that anchor - the first change OUTSIDE the reward, after six reward-different arms all collapsed on the identical feet-factor curve (diagnosed 08-11 as warm-start out-of-distribution drift, not reward) - keeps the feet down and lets the paid stand income finally stick. ONE change vs cw-stand-rsi3: train.bc_anchor_coef=1.0 (rl_move/sim/bc_anchor.py, spec green: 10/10 tests, rise bank unaffected 23-pass re-run, MJX pod smoke anchor loss 0.198->0.04). Prediction-if-true: env/rise_feet_factor stops collapsing (all six prior arms: 0.87->~0.17 by the 25% mark) and env/rise_score leaves the 0.01-0.02 floor; harness rise valid_plant >0 from flat starts. Prediction-if-false: feet-factor collapses on the same curve despite train/bc_anchor_loss staying low - supervision at rollout states cannot beat the drift either, and lever (b) structural height-contact coupling is next. Strongest alternative: anchor holds tracking but the policy goes trajectory-locked/brittle - visible as det replay success with 0 sto success; that still passes discovery (mechanism found) and hardening adds a coef anneal.

**gate**: env/rise_feet_factor holds >=0.5 through 2M (vs 0.87->0.17 by ~500k in all six prior arms); train/bc_anchor_loss sustained <0.1; env/rise_score off the 0.01-0.02 floor and climbing; harness at 2M: rise valid_plant >=1/6 (det or sto) from flat starts, RSI off at eval. Pre-registered kill: feet-factor <0.4 by 500k = anchor refuted, stop, lever (b) next.

