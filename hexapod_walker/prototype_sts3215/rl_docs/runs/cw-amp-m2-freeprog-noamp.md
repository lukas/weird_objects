# cw-amp-m2-freeprog-noamp

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-22T13:40:23+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-pilot-noamp

**wandb_id**: ova8jg2e

**hardware_ready**: False

**hypothesis**: Can a from-scratch policy discover real stepping once standing still stops paying? The 40M M2 pilot pair proved the legacy walk reward's optimum is a statue (one leg triad planted, one airborne; ALL reward growth was stand-income). This control re-runs the no-AMP pilot config from scratch with the bank-calibrated freeprog anti-slip pricing (statue nets -238/ep vs honest gait +558; test_slipwalk_stork_statue_is_priced_out PASS) plus the pre-registered branch-(iii) envelope narrowing (speed 0-0.25 m/s, yaw +/-0.5, pure-walk diet). Prediction-if-true: by 2M the det video shows six legs cycling and real travel (median fwd >= 0.10 m/15 s), unlike the statue fingerprint. Prediction-if-false: freeze/stall fingerprint repeats (cw-nobc-slipwalk1-r1 froze at 2M under this pricing at a fixed command) — which makes the style05 twin the decisive arm: pricing alone insufficient, motion-prior gradient required. Strongest alternative: the idle/park charges destabilize early training into falls instead of stepping.

**gate**: Discovery (2M, DR-0 harness walk mode, 6 det + 6 sto episodes, own cfg): PASS = zero terminations majority of episodes AND median fwd travel >= 0.10 m/15 s AND gait_valid >= 4/6 det with video showing all six legs cycling (statue/flag/stilt = FAIL regardless of scalars). Judged as the matched control for cw-amp-m2-freeprog-style05; no SKILLS/champion updates. Statue fingerprint here + stepping in the style05 twin = the first real style-vs-control win and the Wave-1 unlock evidence.

**verdict**: FAIL (final; dig-in resolved 08-22, c0822-freeprog-suicide-pricing-bank): the topple is SUICIDE ECONOMICS, a pricing defect — with the freeprog stack's per-tick charges at ~-3/tick and reward.term_penalty=0, death was FREE. W&B shows the policy first LEARNED survival (ep_len 28->310 by seg5/6) then flipped to dying fast in q4 (tilt_pitch terminations 59->132/rollout, ep_len 310->256, ep_rew 'recovering' purely by episode shortening) at CONSTANT std 0.367. Scripted topple twin measured +19/ep under the shipped stack vs park -243 / stall -143 / skate -1023 — dying in 1 s was the best-paying behavior in the bank short of real walking. Cheat encoded: test_slipwalk_toppling_fast_is_not_an_escape + term_penalty=400 added to SLIPWALK_OVERRIDES (sized > worst-case discounted survival cost ~-295; bit-exact for surviving behaviors; bank 7/7 PASS, commit d9554b04). Forced log-std anneal REFUTED as the fix (the suicide flip happened at constant std, and survival had already been reached at that std); freeprog/idle charge ramp-in REFUTED (charges were at -2.4/tick from the first log yet survival was still learned mid-run — the collapse came from termination economics, not early destabilization). Fix pair relaunched: same recipe + reward.term_penalty=400, single change.

