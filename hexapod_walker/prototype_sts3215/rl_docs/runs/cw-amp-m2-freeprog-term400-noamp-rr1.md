# cw-amp-m2-freeprog-term400-noamp-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-22T14:32:10+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m2-freeprog-noamp

**wandb_id**: 50sl2j2h

**hypothesis**: Dying must stop paying before a from-scratch policy can discover stepping: the freeprog pair's dig-in proved the topple was suicide economics (per-tick charges ~-3/tick, termination free; scripted 1s topple netted +19/ep vs park -243, and both arms flipped to fast death in q4 after ALREADY learning survival to ep_len 310). Single change vs cw-amp-m2-freeprog-noamp: reward.term_penalty=400, the already-built 08-18 anti-suicide term, sized above the worst-case discounted survival cost (~-295) and bank-verified (topple -381 < park -243 < stall; test_slipwalk_toppling_fast_is_not_an_escape, bank 7/7, commit d9554b04). Prediction-if-true: no q4 termination explosion (tilt terms stay low, ep_len holds near truncation) and the policy stays in the survival regime long enough for the pricing gradient to act; PASS if det video shows six legs cycling with median fwd >= 0.10 m/15s. Prediction-if-false: survives but freezes (park/stall basin, the cw-nobc-slipwalk1-r1 fingerprint) — then the discovery question moves to the style05 twin contrast. Strongest alternative: term fear + charge stack together induce a conservative statue variant the stork test already prices.

**gate**: Discovery (2M, DR-0 harness walk mode, 6 det + 6 sto episodes, own cfg): matched control for the term400-style05 twin. PASS = majority episodes zero terminations AND median fwd travel >= 0.10 m/15s AND gait_valid >= 4/6 det with video showing six legs cycling. Secondary must-hold: q4 termination counts do NOT explode (suicide basin closed). Statue/freeze here + stepping in the style05 twin = the style-vs-control Wave-1 unlock evidence. No SKILLS/champion updates.

**verdict**: Bit-identical accidental dup of cw-amp-m2-freeprog-term400-noamp (same config+seed). Independently confirms: 0/12 terminations DR-0 (suicide fix holds), median fwd travel 0.0315m/15s (need 0.10m) FAIL, gait_valid 5/6 det, slip 6.4-22.2/m. Repro-replicate, not a separate hypothesis test -- see primary noamp verdict for the reward-economics writeup.

