# cw-arch-hist16-dep1-c1-joyfullcurr13-v7-hz100

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-24T17:59:41+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1

**hypothesis**: Plain English: the operator ordered every new model to train at a true 100 Hz control cadence and asked for a 100 Hz retrain of the current best joystick recipe; this run converts the V7 certfreeze recipe (stress-diversified WALKCURR_BUCKETS_V7 ladder with in-place turning + reversals, cert-only freeze, k_walk_stop_current=2.0) to control.hz=100 with the PHYSICAL slew contract preserved (safety.max_delta_q_deg=0.375 = 37.5 deg/s), warm-started from the stable seed0 parent ppo_goal_cw_arch_hist16_dep1_c1.zip because the still-training V7 sibling has no checkpoint yet. RATE-CONVERSION CAVEAT: 40M ticks at 100 Hz is 1/4 the simulated seconds of a 25 Hz 40M run under the current step cap, so this is a rate-conversion experiment, not a 160M-tick equivalence. Prediction-if-true: the recipe learns at 100 Hz -- reward and frontier promotions rise together and 100 Hz evals (side/rear/turn/reversal following, slip, over_current) approach the 25 Hz lineage. Prediction-if-false: reward rises while gate evals stay flat/regress -- a reward/eval mismatch at 100 Hz (per-tick action-delta/current pricing is now 4x denser per second), NOT a seed lottery. Strongest alternative: the 25 Hz-trained warm-start transfers so poorly at 4x obs cadence (hist16 window shrinks 640ms->160ms) that the curriculum stalls at b0 with flat reward -- that reads as warm-start-transfer failure; follow-up is a re-paced init or from-scratch arm, not more budget.

**gate**: PASS: training healthy at 100 Hz with reward/eval AGREEMENT (frontier promotions track reward) AND 60s randomized joygate at 100 Hz: falls <=2/48, no over_current cluster, directions followed on side/rear/turn/reversal stress, slip in/near teacher band (<=~2.9/m), video shows all six feet cycling with no leg sacrifice. PARTIAL: genuinely learning (promotions + improving evals) but short of the 25 Hz lineage numbers within 40M ticks (1/4 sim-time) -- fund a continuation per the 08-21 ruling. FAIL: reward rises while gate evals are flat/regressing (100 Hz reward/eval mismatch -- audit per-tick pricing before ANY seed or budget follow-up) or nothing learns (flat reward AND flat evals with this budget).

**failed_reason**: process died; log tail:
_err=-9.7mm hf=0.95 slew=0.93 stop=0.0236 stop_settled=0.0236 settled_frac=1.00  [informational]
[walkcurr-precert] the INITIAL policy fails the survive/walk bar under exact b0 (falls=0.00, prog=0.203 < 0.50) — fix the transplant/obs mapping first rather than training over it (fb_20260818T102844_116d4c item 6)
[1;34mwandb[0m: 
[1;34mwandb[0m: 🚀 View run [33mcw-arch-hist16-dep1-c1-joyfullcurr13-v7-hz100[0m at: [34mhttps://wandb.ai/l2k2/hexapod-balance/runs/ro125ysb[0m
[1;34mwandb[0m: Find logs at: [1;35mwandb/run-20260824_180047-ro125ysb/logs[0m
/usr/local/lib/python3.11/multiprocessing/resource_tracker.py:254: UserWarning: resource_tracker: There appear to be 82 leaked shared_memory objects to clean up at shutdown
  warnings.warn('resource_tracker: There appear to be %d '


