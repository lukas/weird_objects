# cw-amp-m2-freeprog-term400-style05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T14:26:42+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m2-freeprog-style05

**wandb_id**: u3rmvqly

**hypothesis**: With the suicide basin closed (reward.term_penalty=400, the single change vs cw-amp-m2-freeprog-style05; dig-in proved both freeprog arms learned to die fast because termination was free while being alive cost ~-1.4 to -3/tick), does the AMP style reward supply the stepping-discovery gradient that pricing alone lacks? This re-runs the original Wave-1 fork — which the suicide basin short-circuited before the style channel could matter — under a stack where survival is strictly cheaper than death (bank 7/7 incl. the new topple test, commit d9554b04). Prediction-if-true: this arm steps (six legs cycling, median fwd >= 0.10 m/15s) while the matched term400-noamp control freezes/statues — the first real style-vs-control win. Prediction-if-false: both freeze identically (style gradient too weak -> raise style weight or disc-reward shaping next, not more steps). Strongest alternative: both step (pricing was the whole story; AMP buys gait quality only, judged on video).

**gate**: Discovery (2M, DR-0 harness walk mode, 6 det + 6 sto episodes, own cfg): PASS = median fwd travel >= 0.10 m/15s AND gait_valid >= 4/6 det AND video shows six legs cycling, judged RELATIVE to cw-amp-m2-freeprog-term400-noamp at matched budget. AMP health side-gate: amp/d_real > amp/d_fake unsaturated, style_reward_mean not pinned 0/1 >80% of run. Secondary must-hold: no q4 termination explosion. Stepping here + freeze in twin unlocks Wave-1 sizing. No SKILLS/champion updates.

