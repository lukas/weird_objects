# cw-arch-hist16-dep1-c1-joyfullcurr15-v8-hz100-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-24T22:49:24+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr13-v7-hz100-r2

**hypothesis**: Plain English: the just-verdicted hz100-r2 100Hz rate-conversion arm FAILed but was confounded -- it trained on WALKCURR_BUCKETS_V7, whose diet-scope bug (turn/reversal stress extras leaking into the still-front-cone b1 entry cert bucket) was already root-caused and fixed as V8 in a parallel 25Hz lineage (certfreeze-v8) AFTER this 100Hz run had already launched. This run is the same warm-start/reward/100Hz stack with the single lever --walk-curriculum-version 7->8 (byte-identical bridge/front45 rungs, stress diet now starts at side90_20s where it belongs), giving a confound-free read of whether 100 Hz control itself can clear the b1 stop-settle cert and avoid the leg-3 lock. Prediction-if-true: frontier promotes past b1 (matching certfreeze-v8's own 25Hz read) and the held-out joygate falls drop toward the <=2/48 cap. Prediction-if-false (same signature persists): the leg-lock/joygate-fall pathology is genuinely rate-sensitive, not diet-sensitive, and needs its own 100Hz-specific dig-in.

**gate**: PASS: frontier promotes past b1 to >=b3 AND held-out joygate falls <=2/48 AND DR-0 det gait_valid >=5/6 with no leg-3 sacrifice AND reward/frontier/eval improve together. PARTIAL: frontier promotes and joygate improves (fewer falls / better dir_err) without fully clearing caps -- fund a continuation per the 08-21 ruling. FAIL: frontier still stuck at b1 and/or joygate falls stay >=4/48 with the same leg-3 lock -- points at a genuine 100Hz-specific mechanism (denser per-tick action-delta/current pricing or a control-rate/reflex-timing issue), not the diet, needing its own dig-in.

**failed_reason**: run never appeared as 'running' in W&B within 240s

