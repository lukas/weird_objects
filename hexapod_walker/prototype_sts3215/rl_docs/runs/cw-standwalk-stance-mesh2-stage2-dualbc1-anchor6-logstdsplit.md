# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor6-logstdsplit

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-27T03:52:30+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor4-stdanneal

**wandb_id**: uhbf737q

**hypothesis**: Plain sentence: give the stance core its own separate exploration-noise knob so cooling it toward -4.0 (which already fixes hold) no longer has to also cool the walk core (which is what wrecked walk in anchor4-stdanneal and still failed to help hold at -1.0/-2.0 in the anchor5-stdmild bracket). Single mechanism change vs anchor4-stdanneal (same coef=3.0/isolate_update=1 dual-core recipe, same --init-from anchor2 checkpoint, same --log-std-final -4.0 --log-std-anneal-frac 0.5 schedule): add --gru-dual-log-std-split (a second learnable log_std_b for core B/stance, mixed per-tick by the same mode gate as the mean/value) and --log-std-anneal-core stance so the -4.0 anneal target hits ONLY log_std_b, leaving the walk core's own log_std to keep training normally under PPO's own gradient. Prediction-if-true: hold/sto DR-0 termination collapses like anchor4-stdanneal's own result (6/6->0-2/6) AND walk stays gait_valid>=5/6 with prog_ratio in the healthy 0.2-0.4 band (no anchor1/anchor4-class leg-sacrifice freeze) on BOTH seeds -- the FULL PASS the whole 4-arm magnitude bracket could not reach. Prediction-if-false: walk still degrades even with its own log_std untouched -- would mean the earlier walk-destruction wasn't purely an exploration-noise-starvation effect (e.g. a value-function or reward-shaping interaction instead), pointing the dig-in at the critic/reward side next.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition or require mature gait. Same joint hold/rise/lower/walk DR-0 det+sto + own-DR(0.5) panel as anchor2/3/4-stdanneal/5-stdmild. FULL PASS = WALK-SURVIVES (det gait_valid >=5/6 both seeds, no anchor1/anchor4-class 3+-leg-sacrifice freeze, prog_ratio >=~0.2) AND HOLD-HELPS-FULL (hold/sto DR-0 termination <=2/6, matching anchor4-stdanneal's own hold result) on BOTH seeds -- this is the FULL PASS the entire dose-bracket grid (anchor4-stdanneal, anchor5-stdmild1/2 x2 seeds, 4 arms) failed to reach with a single shared log_std. PARTIAL if hold improves less than anchor4-stdanneal's own result but still beats the anchor2/3 6/6 baseline while walk survives -- promote to a longer/tuned-anneal-target follow-up. FAIL if walk still shows the anchor4-class catastrophe on either seed despite the split (closes the exploration-noise-starvation theory of the walk failure, points at critic/reward-shaping instead) or if hold shows zero improvement (the split mechanism itself is broken/not wired correctly -- check for a code defect before any further arm).

