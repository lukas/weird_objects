# cw-arch-hist16-dep1-c1-joyfullcurr15-v8-hz100-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-24T23:05:05+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr13-v7-hz100-r2

**wandb_id**: 4tddgstm

**hypothesis**: Plain English: the just-verdicted hz100-r2 100Hz rate-conversion arm FAILed but was confounded -- it trained on WALKCURR_BUCKETS_V7, whose diet-scope bug (turn/reversal stress extras leaking into the still-front-cone b1 entry cert bucket) was already root-caused and fixed as V8 in a parallel 25Hz lineage (certfreeze-v8) AFTER this 100Hz run had already launched. This run is the same warm-start/reward/100Hz stack with the single lever --walk-curriculum-version 7->8 (byte-identical bridge/front45 rungs, stress diet now starts at side90_20s where it belongs), giving a confound-free read of whether 100 Hz control itself can clear the b1 stop-settle cert and avoid the leg-3 lock. Prediction-if-true: frontier promotes past b1 (matching certfreeze-v8's own 25Hz read) and the held-out joygate falls drop toward the <=2/48 cap. Prediction-if-false (same signature persists): the leg-lock/joygate-fall pathology is genuinely rate-sensitive, not diet-sensitive, and needs its own 100Hz-specific dig-in.

**gate**: PASS: frontier promotes past b1 to >=b3 AND held-out joygate falls <=2/48 AND DR-0 det gait_valid >=5/6 with no leg-3 sacrifice AND reward/frontier/eval improve together. PARTIAL: frontier promotes and joygate improves (fewer falls / better dir_err) without fully clearing caps -- fund a continuation per the 08-21 ruling. FAIL: frontier still stuck at b1 and/or joygate falls stay >=4/48 with the same leg-3 lock -- points at a genuine 100Hz-specific mechanism (denser per-tick action-delta/current pricing or a control-rate/reflex-timing issue), not the diet, needing its own dig-in.

**verdict**: V8 diet-scope fix DOES generalize to 100Hz (frontier cleared b1, reached b3/side90_20s, promotions=3 -- matches certfreeze-v8's 25Hz read) but the run misses its own pre-registered PASS bar on every remaining clause: held-out 60s joygate falls 7/48 (cap <=2, also >=4/48 FAIL threshold), DR-0 det gait_valid only 2/6 (need >=5/6), own-DR(0.5) det 1/6 -- and leg 3 is named in the sacrificed set in 4/6 DR-0-det, 2/6 DR-0-sto, 3/6 own-DR-det and 1/6 own-DR-sto episodes (video walk_det_1/2/4/5 all show one rigid held-up leg while the other five cycle; walk_det_0/3 by contrast show clean six-leg cycling with real forward progress, prog 0.53-1.46x clone-scale, confirming the policy CAN produce a healthy gait, just not reliably). Zero falls/terminations across all 24 DR-0/own-DR episodes (terms=0 every group) -- the pathology here is intermittent leg-3 freeze, not topple; the held-out joygate's longer 60s/direction-changing sessions are what convert the same intermittent freeze into 7 real falls. ep_rew_mean quarters 1289.8/606.9/1297.0/1066.0 -- dipped then partially recovered, ending below its own Q3 peak while gait_valid/falls stayed bad the whole run: reward plateaued at a mediocre optimum, not still climbing, so per the 08-21 ruling this reads as a genuine stuck mechanism (matching FAIL) rather than undertrained. Matches this run's own pre-registered FAIL branch ('frontier still stuck at b1 and/or joygate falls stay >=4/48 with the same leg-3 lock') on the joygate/leg-3 half even though frontier itself cleared b1. No same-recipe continuation: the decisive lever for leg-sacrifice (reward.walk_gait_gate, absent from this V8/100Hz recipe) is already being tested head-to-head by the concurrent gaitgate-scratch1/-seed1/-seed2 trio (from-scratch, gate baked in from step 0) -- this run's remaining value was confirming V8's diet fix generalizes to 100Hz (it does), which is now closed. PRESTAGE NOTE: the watcher's pod_eval.py process for this run died silently after both on-pod passes finished (no SYNCED line, no local copy) -- recovered manually via kubectl cp; report.json/videos now at logs/ckpt_eval/cw_arch_hist16_dep1_c1_joyfullcurr15_v8_hz100_r2_{gate,owncfg}/, W&B 4tddgstm.

