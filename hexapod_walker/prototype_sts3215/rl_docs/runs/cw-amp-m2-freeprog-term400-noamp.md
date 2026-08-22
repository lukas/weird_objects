# cw-amp-m2-freeprog-term400-noamp

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-22T14:29:07+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-amp-m2-freeprog-noamp

**wandb_id**: yvpeo6ay

**hypothesis**: Dying must stop paying before a from-scratch policy can discover stepping: the freeprog pair's dig-in proved the topple was suicide economics (per-tick charges ~-3/tick, termination free; scripted 1s topple netted +19/ep vs park -243, and both arms flipped to fast death in q4 after ALREADY learning survival to ep_len 310). Single change vs cw-amp-m2-freeprog-noamp: reward.term_penalty=400, the already-built 08-18 anti-suicide term, sized above the worst-case discounted survival cost (~-295) and bank-verified (topple -381 < park -243 < stall; test_slipwalk_toppling_fast_is_not_an_escape, bank 7/7, commit d9554b04). Prediction-if-true: no q4 termination explosion and the policy stays in the survival regime long enough for the pricing gradient to act; PASS if det video shows six legs cycling with median fwd >= 0.10 m/15s. Prediction-if-false: survives but freezes (park/stall basin, the cw-nobc-slipwalk1-r1 fingerprint) — then the discovery question moves to the style05 twin contrast. Strongest alternative: term fear + charge stack induce a conservative statue variant the stork test already prices.

**gate**: Discovery (2M, DR-0 harness walk mode, 6 det + 6 sto episodes, own cfg): matched control for the term400-style05 twin. PASS = majority episodes zero terminations AND median fwd travel >= 0.10 m/15s AND gait_valid >= 4/6 det with video showing six legs cycling. Secondary must-hold: q4 termination counts do NOT explode (suicide basin closed). Statue/freeze here + stepping in the style05 twin = the style-vs-control Wave-1 unlock evidence. No SKILLS/champion updates.

**verdict**: Suicide fix WORKED (term_penalty=400: 0/12 terminations DR-0 det+sto, matches rr1 repro) but the walk gate FAILS on progress: median fwd travel 0.026m/15s (need 0.10m), gait_valid 3/6 det (rr1 5/6), high slip 6.6-22.7/m. W&B: reward_walk_prog=0 by freeprog design (expected), but the freeprog cross/backward penalty (env/reward_walk_freeprog_pen) sits flat ~-1.4 to -1.8/tick with NO improvement q1-end to q4 (per-tick reward_per_tick_ema worsens -1.17->-2.84 then flattens) -- flat reward + flat task metric = genuine stuck point, not reward-still-rising. This is the predicted-if-false outcome (park/stall basin, cw-nobc-slipwalk1-r1 fingerprint): six legs do cycle (not a pure statue) but slip in place without organizing into net commanded-direction travel. Matched to style05 twin (another cycle) to close the Wave-1 style-vs-control fork.

