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

**verdict**: FAIL, same failure mode as its matched noamp control -- the AMP style reward did NOT rescue this pricing regime. Reward declined the whole 2M budget (quarters -33->-216->-398->-372/ep; the partial q4 recovery tracks ep_len_mean SHRINKING 312->219 over the same window, i.e. episodes are dying faster, not behaving better -- a possible 'die fast to cap penalty exposure' artifact, not genuine improvement). Held-out DR-0 gate: 11/12 episodes terminated tilt_pitch/tilt_roll within 1-2s; forward travel 0.014-0.081 m/15s, nowhere near the 0.10 m bar; slip 5.2-12.4/m. Video: same rapid topple-from-standing fingerprint as noamp, not the predicted 'style buys stepping' outcome. AMP mechanism itself stayed healthy the whole run (d_real 0.77 vs d_fake -0.94, unsaturated, style_reward_mean 0.093, 124 disc updates) -- the style channel simply cannot out-bid a ~-1.4 to -2.8/tick freeprog+loadslip penalty that is maxed out from step zero regardless of AMP. Wave-1 sizing fork is UNRESOLVED: neither predicted branch (stepping vs freeze) occurred in either arm -- a third failure mode (catastrophic early instability) requires root-causing before another discovery arm is funded. DIG-IN queued alongside cw-amp-m2-freeprog-noamp.

