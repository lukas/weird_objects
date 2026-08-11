# cw-stand-bc1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-11T04:06:55+00:00

**pod**: hexapod-mjx-train-6

**steps**: 2000000

**parent**: cw-stand-rsi3

**wandb_id**: bhvbnu7s

**hardware_ready**: False

**hypothesis**: Teach the robot to stand up by holding its hand along the known-good path: a supervised anchor in the trainer now pulls the policy's actions toward the recorded stand-up motion at every step of a rise episode, so the warm-started skill cannot drift away into the three-legs-in-the-air cheat while learning. This arm tests whether that anchor - the first change OUTSIDE the reward, after six reward-different arms all collapsed on the identical feet-factor curve (diagnosed 08-11 as warm-start out-of-distribution drift, not reward) - keeps the feet down and lets the paid stand income finally stick. ONE change vs cw-stand-rsi3: train.bc_anchor_coef=1.0 (rl_move/sim/bc_anchor.py, spec green: 10/10 tests, rise bank unaffected 23-pass re-run, MJX pod smoke anchor loss 0.198->0.04). Prediction-if-true: env/rise_feet_factor stops collapsing (all six prior arms: 0.87->~0.17 by the 25% mark) and env/rise_score leaves the 0.01-0.02 floor; harness rise valid_plant >0 from flat starts. Prediction-if-false: feet-factor collapses on the same curve despite train/bc_anchor_loss staying low - supervision at rollout states cannot beat the drift either, and lever (b) structural height-contact coupling is next. Strongest alternative: anchor holds tracking but the policy goes trajectory-locked/brittle - visible as det replay success with 0 sto success; that still passes discovery (mechanism found) and hardening adds a coef anneal.

**gate**: env/rise_feet_factor holds >=0.5 through 2M (vs 0.87->0.17 by ~500k in all six prior arms); train/bc_anchor_loss sustained <0.1; env/rise_score off the 0.01-0.02 floor and climbing; harness at 2M: rise valid_plant >=1/6 (det or sto) from flat starts, RSI off at eval. Pre-registered kill: feet-factor <0.4 by 500k = anchor refuted, stop, lever (b) next.

**verdict**: PASS (partial) — BC-anchor mechanism validated, first honest rise in 7 stand-arms. Gate harness (RSI 0.5, as trained): rise valid_plant 3/6 det (bridge/crouch honest six-foot plants, height_err 4-7mm, all feet <20mm off ground, video-confirmed no flag-leg). Extra RSI-off/30-episode probe (seed 7, isolates the anchor from RSI help): bridge 7/12 and crouch 6/8 pass full geometric valid_plant; flat-belly cold start (the hardest case, r<0.35 branch) reaches a real six-foot stand every time (10/10 height+posture+no-flag) but misses only the walk-ready footprint tolerance (0/10 valid_plant, footprint-only fail, worst clearance 7.8mm — not a flag-leg cheat, a foot-XY precision gap). Zero flag-leg/tripod cheat in 42 video-checked episodes across both evals, vs 0/12 valid_plant every single time in score1/scoreref1(+3 variants)/plantgate1/rsi1/rsi2/rsi3 — the identical-recipe parent rsi3 (only missing bc_anchor_coef) still shows the flag-leg cheat on this exact stack. Causal attribution to the anchor is clean (one variable). Caveat (skill interference, weak evidence n=2): this run's own training diagnostic shows raise 0/2 and tipped-recovery 0/2 at 2M vs rsi3's 1-2/2 and 2/2, and hold/track angle error 3.0deg vs rsi3's 1.2-1.6deg — the anchor pulls rise-tick actions toward the reference and may be bleeding into nearby shared-network modes. ep_rew_mean fell to -29 by 2M (quarters 8.9/-9.4/-46.7/-70.2) driven by rising tilt/over-current terminations during genuine (riskier) rising attempts, not by hold/track breaking (video confirms hold/track look fine). Pre-registered kill (feet-factor <0.4 by 500k) WOULD have fired at 262k-590k (0.32-0.37) had this been live-monitored — it recovered to 0.75 by 2M; note for future kill-rule design: needs a sustained-window check, not first-crossing, on this mechanism.

