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

**verdict**: PASS (partial) — BC-anchor mechanism validated, first honest rise in 7 stand-arms. Gate harness (RSI 0.5, as trained): rise valid_plant 3/6 det (bridge/crouch honest six-foot plants, height_err 4-7mm, all feet <20mm off ground, video-confirmed no flag-leg). Extra RSI-off/30-episode probe (seed 7): bridge 7/12 and crouch 6/8 pass full geometric valid_plant; flat-belly cold start reaches a real six-foot stand every time (10/10) but misses only the walk-ready footprint tolerance (0/10, footprint-only fail, not a flag-leg cheat). Zero flag-leg/tripod cheat in 42 video-checked episodes, vs 0/12 valid_plant every time in the identical-minus-anchor parent rsi3 — clean one-variable attribution. CORRECTION (found reviewing cw-stand-bc1-hard1, 08-11): the initial 'hold/track look fine, video confirms' read was a SPARSE-FRAME-STRIP miss. The report.json duty_cycle/swing_count/end_clear_mm fields (not checked at first pass) show hold/track are NOT quiet stands here: alternating legs cycle continuously (duty ~0.85-0.9/0.06-0.09, 6-14 swings per 15s episode) ending 12-50mm elevated (tail-mean) — hold/track success 0/6 both modes. This pathology predates the anchor concern list (it is not the n=2 raise/tipped hint, it's a real, quantified stepping-during-hold pattern) and WORSENS at 10M steps (bc1-hard1: 100-161mm). Lesson for future triage: check duty_cycle/swing_count/end_clear_mm for stand-line modes, not just valid_plant + a 10-frame strip. This does not change the RISE verdict (still PASS-partial, the mechanism genuinely fixes the flag-leg cheat) but the interference cost is real and pre-existing, not a new anchor side-effect alone — likely a hold/track income-pricing gap (stepping isn't charged) that the anchor's shared-network pull amplifies. Two follow-ups launched: cw-stand-bc1-hard1 (10M hardening — rise consolidated further to 5/6 det valid_plant, hold/track cycling WORSENED, see its own verdict) and cw-stand-bc1-coef03 (coef 0.3 dose-check — FAILED decisively, valid_plant 0/16, keep coef>=1.0).

