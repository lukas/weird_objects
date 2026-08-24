# cw-arch-hist16-dep1-c1-joyfullcurr16-certfreeze-v9

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-24T23:27:35+00:00

**pod**: hexapod-mjx-train-5

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr14-certfreeze-v8

**hypothesis**: Plain English: certfreeze-v8 (verdicted this cycle) proved the bucket-scope fix works -- frontier leaves b1, reaches b3 (side90 opens) -- but then plateaus AT b3 for the rest of the 40M-step run with the identical stuck-stop-cert signature v7 showed at b1. Root cause (code-read, this cycle): the cert-time hold supervisor (_walk_stop_freeze_override) exempts any tick with wz_ref!=0, but a wz-diet bucket's stop_frac and wz_zero_frac draws are independent rng calls on the SAME resampled segment, so ~half of a nominal 'stop' segment's ticks also carry wz!=0 in any bucket carrying the diet (side90_20s onward) and are therefore exempt from the very freeze meant to clear the 0.015 m/s bar, while the legacy stop_speed_m_s metric counts every such tick anyway -- structurally guaranteeing the cert can never clear regardless of policy quality. This run tests the ONE fix: --walk-curriculum-version 8->9 (WALKCURR_BUCKETS_V9, byte-identical bucket scope/diet to V8 -- built+tested this cycle, test_walk_curriculum.py 56/56 incl. 4 new V9 tests, test_walkcurr_mjx.py 19/19, test_walk_stop_settle_metric.py 6/6 incl. 2 new stop_speed_pure_m_s tests -- except every gated bucket is certified on the new stop_speed_pure_m_s metric instead, which excludes the same wz!=0 ticks the freeze does, so the cert and the freeze finally agree about what counts as 'commanded still'.

**gate**: PASS: frontier promotes past b3 to at least b5 (rear135 opens) AND held-out 60s joygate falls <=2/48 AND DR-0 det gait_valid stays 6/6 no leg sacrifice. PARTIAL: frontier promotes past b3 but joygate/DR-0-gait doesn't fully clear (genuine trade, or improvement without clearing). FAIL: frontier still stalls at b3 (the cert-metric fix didn't touch the mechanism, or turning-in-place genuinely induces real translational drift the freeze can't suppress) -- points next at building a dedicated turn-in-place drift/stillness gate instead of a metric-alignment fix.

**refused_reason**: new PPO launches must train at control.hz=100 (got 25); use a separate explicit legacy/eval path for old 25 Hz checkpoints

