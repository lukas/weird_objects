# cw-arch-hist16-dep1-c1-joyfullcurr10-stopsettle-probe

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-24T08:26:48+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr9-stopcur2

**hypothesis**: Plain English: audit whether the V6 walkcurr cert bar (mean stop-tick speed <=0.015 m/s) is even measuring the same quantity the reward has ever been priced to optimize. The cert probe (_walk_probe_tick) has always averaged EVERY stop tick from the very first one of a commanded-stop segment; the reward's own stop charges (k_walk_stop_charge/k_walk_stop_current) instead ramp their multiplier 0->1 over reward.walk_stop_grace_s=0.4s specifically because that transient is an unavoidable physical deceleration, not creep (the joyfullcurr7 finding). This is a precert-only DRY RUN (no PPO training, exits before any training step) reading the already-trained joyfullcurr9-stopcur2 checkpoint under a new, additive-only diagnostic (goal.walk_stop_settle_s, default 0.0 = bit-exact legacy, landed+unit-tested this cycle in test_walk_stop_settle_metric.py) that excludes exactly the reward-exempted 0.4s grace window from a NEW stop_speed_settled_m_s field, leaving the legacy stop_speed_m_s the cert gate actually reads completely untouched.

**gate**: Diagnostic only, no pass/fail gate on this run itself. If stop_speed_settled_m_s reads meaningfully lower than stop_speed_m_s (toward/under 0.015): the CERT METHODOLOGY (not the reward) is the blocker on the joyfullcurr9/10 stop-speed-charge-dose lever -- next step is amending the cert's own measurement to match the grace window rather than any further stop-charge dosing. If stop_speed_settled_m_s stays close to the raw stop_speed_m_s (little change): the residual creep is a genuine POST-grace floor -- the stop-speed-charge-dose lever is refuted both by price (joyfullcurr9/10 dose ladder) and by measurement methodology, and the next lever must be structural (an explicit freeze/hold controller on stop commands) rather than more reward pricing.

**verdict**: Cert-methodology audit, not a training run (--walkcurr-precert-only: builds the env, reads the joyfullcurr9-stopcur2 checkpoint's b0/b1 precert, exits before any PPO step; ledger auto-status shows FAILED only because the launcher's W&B-running-state poll never caught the run mid-flight -- it finished in ~15s, rc 0, 'precert-only mode: exiting PASS' in the trainlog. Correcting the mis-tag here). Built goal.walk_stop_settle_s (default 0.0, additive-only, bit-exact-when-absent, 4/4 new unit tests in test_walk_stop_settle_metric.py) to test whether the cert's stop_speed_m_s (which averages EVERY stop tick from the first one) differs from a version that excludes the same 0.4s grace window the reward's own stop charges already exempt (walk_stop_grace_s) -- the pre-registered audit from the chg2/chg4 dose-lever closure. RESULT: at settle_s=0.4 on the stopcur2 checkpoint's b1 bucket, stop_speed_m_s=0.0326 vs stop_speed_settled_m_s=0.0311 (settled_frac=0.80, i.e. 80% of stop ticks were already past the grace window) -- only a 5% drop, both numbers ~2x the 0.015 cap. This DEFINITIVELY answers the audit: the residual creep is a genuine POST-grace steady-state floor, NOT a cert-methodology artifact (excluding the exact window the reward already discounts barely moves the number) and NOT primarily a decel transient (80% of measured ticks were already outside any transient window). Combined with chg2/chg4's price-insensitivity (0/1/2/4x dose, same 0.03-0.045 band): the stop-speed-charge mechanism class is now closed by BOTH price and methodology. Next lever must be structural (e.g. an anchor/hold gate on loaded-foot position during stop ticks, reusing the existing walk_anchor_gate pattern which today only fires while s_ref>1e-3) rather than any further stop-charge dosing or cert-measurement change -- recorded as the joystick V6 ladder's next specified-but-unbuilt item.

**failed_reason**: run never appeared as 'running' in W&B within 240s

