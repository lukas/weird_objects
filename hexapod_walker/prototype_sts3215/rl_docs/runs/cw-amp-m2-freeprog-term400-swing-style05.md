# cw-amp-m2-freeprog-term400-swing-style05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-22T17:13:09+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m2-freeprog-term400-style05-v2

**wandb_id**: 3zpdrkdn

**hypothesis**: Style-vs-control twin to cw-amp-m2-freeprog-term400-swing-noamp: does the AMP style channel (teacher_v2 lib, 0.5/0.5, the already-verdicted-FAIL style05-v2 config) add anything ON TOP of the new k_walk_swing income gradient, now that command complexity, term-penalty, std-anneal, staging, style-dose, and RSI are all closed as levers for this statue and swing is the first mechanism to raise real-gait income in the bank (see swing-noamp's hypothesis for the full ladder + bank numbers)? Single lever vs cw-amp-m2-freeprog-term400-style05-v2 (FAIL, statue): reward.k_walk_swing=1.0, everything else (teacher_v2 lib, style/task 0.5/0.5, term_penalty=400) unchanged. Prediction-if-true: this arm's fwd travel clears noticeably ABOVE its swing-noamp twin's -- the style channel supplies coordination (which LEG to swing, alternating tripod phase) that a direction-agnostic swing bonus alone cannot, so AMP + swing complement each other. Prediction-if-false: both swing arms land at the same fwd-travel reading -- swing supplies the whole gradient and AMP's contribution to this family stays unproven at this pricing/budget. Same pre-registered single-leg-farm cheat to watch as the noamp twin, PLUS AMP's own pre-registered cheat (in-place teacher mimicry at style_reward_mean pinned near 1.0 -- did not fire in style05-v2/stylew2-v2, watch again here since the swing bonus changes what states get visited).

**gate**: Discovery (2M, DR-0 harness walk mode, 6 det + 6 sto, own cfg), judged RELATIVE to cw-amp-m2-freeprog-term400-swing-noamp (twin) and the noamp/style05-v2/stylew2-v2/fixedcmd statue family at matched budget: PASS = median det fwd travel >= 0.10 m/15s AND gait_valid >= 4/6 det AND no sacrificed legs AND video shows six legs cycling with net displacement. INFORMATIVE = det fwd travel clears meaningfully above 0.03m OR gait_valid > 0/6 for the first time, read RELATIVE to swing-noamp (if this arm clears swing-noamp's reading by a wide margin, AMP is adding a real coordination signal on top of the swing gradient; if indistinguishable, AMP's contribution stays unproven here). Cheat check same as swing-noamp (single-leg-farm) PLUS in-place-teacher-mimicry (style_reward_mean pinned near 1.0 with fwd travel ~0) -- either voids a PASS/INFORMATIVE read regardless of return.

**verdict**: FAIL, matches its swing-noamp twin (statistically indistinguishable) -- the AMP style channel adds nothing on top of k_walk_swing either. DR-0 gate: det fwd med 0.032m/15s (bar 0.10m, twin 0.026m), gait_valid 5/6 (1 leg sacrificed once), slip 8.0-11.2/m; sto fwd med 0.026m, gait_valid 6/6, slip 4.4-23/m. Video near-identical shuffle fingerprint to swing-noamp (imbalanced duty across legs, no organized tripod, no net displacement). W&B: ep_rew_mean fell every quarter (-135/-352/-546/-517, never rising), env/reward_walk_freeprog_pen pinned -1.45 to -1.50/tick, env/reward_swing pinned ~0.05-0.06/tick (identical to the noamp twin -- AMP style income did not change the swing income or unlock anything it did not already have). style_reward_mean stayed low (consistent with every prior style arm in this family -- the discriminator never got the chance to matter). CLOSES the k_walk_swing x AMP-style combination for this family at this dose; the style-vs-control question this arm was built to answer (do they complement each other) reads NO at 2M/dose=1.0. Follow-up: cw-amp-m2-freeprog-term400-rsi1-{noamp,style05b} tests an INITIAL-STATE lever (RSI mid-gait spawn) instead of another reward-side income channel.

