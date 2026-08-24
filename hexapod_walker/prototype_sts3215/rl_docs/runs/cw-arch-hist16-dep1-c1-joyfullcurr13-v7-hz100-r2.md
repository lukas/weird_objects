# cw-arch-hist16-dep1-c1-joyfullcurr13-v7-hz100-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-24T18:12:37+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1

**wandb_id**: a7aanvq3

**hypothesis**: Plain English: the operator ordered every new model to train at a true 100 Hz control cadence and asked for a 100 Hz retrain of the current best joystick recipe; this run converts the V7 certfreeze recipe (stress-diversified WALKCURR_BUCKETS_V7 ladder with in-place turning + reversals, cert-only freeze, k_walk_stop_current=2.0) to control.hz=100 with the PHYSICAL slew contract preserved (safety.max_delta_q_deg=0.375 = 37.5 deg/s), warm-started from the stable seed0 parent ppo_goal_cw_arch_hist16_dep1_c1.zip because the still-training V7 sibling has no checkpoint yet. Attempt 1 (same name minus -r2) died FAIL-CLOSED at the inherited walkcurr-precert bar (init prog=0.203 < 0.50 at 100 Hz, 0 falls) -- that bar exists to catch broken transplants, and 0.203-with-no-falls shows the mapping is intact and the shortfall is the rate mismatch this run exists to retrain; cert-at-init/precert flags are dropped for this arm so the frontier starts honestly at b0. RATE-CONVERSION CAVEAT: 40M ticks at 100 Hz is 1/4 the simulated seconds of a 25 Hz 40M run -- a rate-conversion experiment, not a 160M-tick equivalence. Prediction-if-true: reward and frontier promotions rise together and 100 Hz evals (side/rear/turn/reversal following, slip, over_current) approach the 25 Hz lineage. Prediction-if-false: reward rises while gate evals stay flat/regress -- reward/eval mismatch at 100 Hz (per-tick action-delta/current pricing 4x denser per second), NOT a seed lottery. Strongest alternative: warm-start transfer is too poor at 4x cadence (hist16 window 640ms->160ms) and the curriculum stalls at b0 with flat reward -- warm-start-transfer failure; follow-up is a re-paced init or from-scratch arm, not more budget.

**gate**: PASS: training healthy at 100 Hz with reward/eval AGREEMENT (frontier promotions track reward) AND 60s randomized joygate at 100 Hz: falls <=2/48, no over_current cluster, directions followed on side/rear/turn/reversal stress, slip in/near teacher band (<=~2.9/m), video shows all six feet cycling with no leg sacrifice. PARTIAL: genuinely learning (promotions + improving evals) but short of the 25 Hz lineage numbers within 40M ticks (1/4 sim-time) -- fund a continuation per the 08-21 ruling. FAIL: reward rises while gate evals are flat/regressing (100 Hz reward/eval mismatch -- audit per-tick pricing before ANY seed or budget follow-up) or nothing learns (flat reward AND flat evals with this budget).

