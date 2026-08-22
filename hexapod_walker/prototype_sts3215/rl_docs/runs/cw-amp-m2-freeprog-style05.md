# cw-amp-m2-freeprog-style05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-22T13:44:15+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m2-pilot-style05

**wandb_id**: p2t7k88y

**hardware_ready**: False

**hypothesis**: Does the AMP style reward supply the discovery gradient that pricing alone lacks? With the statue exploit priced out (bank-calibrated freeprog stack: stork statue -238/ep vs honest gait +558) the two known failure modes are now separated: the legacy-priced pilots proved style income (~0.03/tick effective) cannot outbid statue income (~1.9/tick), and cw-nobc-slipwalk1-r1 proved aligned pricing WITHOUT a motion prior still freezes at 2M (PPO never discovers stepping). This arm = repriced task (freeprog stack, pure-walk diet, narrowed envelope speed 0-0.25 / yaw +/-0.5) + the healthy AMP mechanism (fresh discriminator, task/style 0.5/0.5), from scratch per the 08-22 init-basin rule. Prediction-if-true: by 2M det video shows six legs cycling with real travel (median fwd >= 0.10 m/15 s) while the matched noamp twin repeats the freeze — the first real style-vs-control signal and the Wave-1 unlock. Prediction-if-false: freeze identical to the twin (style gradient too weak even without statue competition -> raise style weight or disc-reward shaping next, not more steps). Strongest alternative: both arms step (pricing was the whole story; AMP buys only gait QUALITY, judged at the visual comparison).

**gate**: Discovery (2M, DR-0 harness walk mode, 6 det + 6 sto episodes, own cfg): PASS = median fwd travel >= 0.10 m/15 s AND gait_valid >= 4/6 det AND video shows all six legs cycling (statue/flag/stilt = FAIL regardless of scalars), judged RELATIVE to cw-amp-m2-freeprog-noamp at matched budget; AMP health side-gate: amp/d_real > amp/d_fake un-saturated, style_reward_mean not pinned 0/1 >80% of run. No SKILLS/champion updates; stepping-here + freeze-in-twin unlocks Wave-1 sizing.

**verdict**: FAIL (final; dig-in resolved 08-22, paired with cw-amp-m2-freeprog-noamp): same root cause — SUICIDE ECONOMICS (per-tick charges ~-1.4 to -3/tick with term_penalty=0 makes death free; scripted 1s topple nets +19/ep vs park -243). Q4 'recovery' CONFIRMED as faster death, not better behavior: ep_rew -430->-268 while ep_len fell 292->230, tilt_pitch terminations TRIPLED 90->241 and truncations fell 164->125 — the reward improvement is episode shortening, full stop. AMP mechanism stayed healthy and is exonerated: the style channel is irrelevant to termination economics it cannot price. Style-vs-control question (Wave-1 fork) remains UNRESOLVED — both arms found the suicide basin before the style gradient could matter; the term_penalty=400 fix pair re-runs the exact contrast. Cheat encoded in bank (test_slipwalk_toppling_fast_is_not_an_escape, 7/7 PASS, commit d9554b04); log-std anneal and charge ramp-in both refuted as fixes (see noamp verdict).

