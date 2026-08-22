# cw-amp-m2-freeprog-term400-style05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-22T14:26:42+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m2-freeprog-style05

**wandb_id**: u3rmvqly

**hypothesis**: With the suicide basin closed (reward.term_penalty=400, the single change vs cw-amp-m2-freeprog-style05; dig-in proved both freeprog arms learned to die fast because termination was free while being alive cost ~-1.4 to -3/tick), does the AMP style reward supply the stepping-discovery gradient that pricing alone lacks? This re-runs the original Wave-1 fork — which the suicide basin short-circuited before the style channel could matter — under a stack where survival is strictly cheaper than death (bank 7/7 incl. the new topple test, commit d9554b04). Prediction-if-true: this arm steps (six legs cycling, median fwd >= 0.10 m/15s) while the matched term400-noamp control freezes/statues — the first real style-vs-control win. Prediction-if-false: both freeze identically (style gradient too weak -> raise style weight or disc-reward shaping next, not more steps). Strongest alternative: both step (pricing was the whole story; AMP buys gait quality only, judged on video).

**gate**: Discovery (2M, DR-0 harness walk mode, 6 det + 6 sto episodes, own cfg): PASS = median fwd travel >= 0.10 m/15s AND gait_valid >= 4/6 det AND video shows six legs cycling, judged RELATIVE to cw-amp-m2-freeprog-term400-noamp at matched budget. AMP health side-gate: amp/d_real > amp/d_fake unsaturated, style_reward_mean not pinned 0/1 >80% of run. Secondary must-hold: no q4 termination explosion. Stepping here + freeze in twin unlocks Wave-1 sizing. No SKILLS/champion updates.

**verdict**: Discovery FAIL against own gate: DR-0 det/sto median fwd travel 0.02m/15s (bar 0.10m), slip 8-9/m, gait_valid 6/6 but video (contact sheet + 12-frame strips, 3 episodes) shows the body NOT translating across the full episode -- a marching-in-place / paddle-creep equilibrium, not a freeze and not a fall. Root cause (W&B trace): term_penalty=400 DID fix the prior suicide-economics failure (ep_len_mean climbs monotonically 13->365 over 2M steps with NO late-run collapse, terminations dominated by truncated not tilt_pitch/roll -- opposite of the prior arms' q4 death-flip). But env/walk_gait_min (continuous gait-quality factor) crashes 1.0->~0.3 by ~500k steps and never recovers, env/reward_walk_prog stays 0 throughout (freeprog-only stack), env/walk_freeprog_score and env/walk_speed stay flat ~0.03 the entire run, and env/reward_loadslip_excess gets MORE negative over training (increasing bill despite near-zero net motion) -- the policy is not learning locomotion, it found a lower-risk micro-stepping basin once falling stopped being free. AMP mechanism stayed healthy throughout (amp/d_real_mean 0.65-0.78 vs d_fake -0.95to-1.0, unsaturated, style_reward_mean 0.03-0.10) but had nothing to rescue. reward_per_tick_ema bottomed at -1.35 (step ~850k) then partially recovered to -1.35->-1.33 (essentially flat, not a real climb) while task metrics stayed dead flat -- this is the RUN_INTERPRETATION_RULES 'genuine FAIL' bucket (task metrics flat with adequate discovery budget), not a rising-reward case. Context (read-only, not verdicting): the noamp/noamp-rr1 twins' own W&B reward is monotonically worse with no tail recovery (-1163/-1162 vs style05's -538) -- suggestive that AMP may be doing SOMETHING here, but that comparison is the twins' own triage to make, not decided by this run alone. NEW pricing lead for the next arm: freeprog's cap (k_walk_freeprog=3, walk_freeprog_cap_m_s=0.05) may be too small relative to k_loadslip_excess=6 risk to reward pushing stride amplitude back up once death stops being the cheap option -- candidate fix is raising the freeprog cap/gain or adding a stride-length/gait_min floor charge, not touching term_penalty again.

