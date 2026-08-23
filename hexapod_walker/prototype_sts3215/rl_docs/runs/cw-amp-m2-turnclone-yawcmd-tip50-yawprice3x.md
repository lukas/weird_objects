# cw-amp-m2-turnclone-yawcmd-tip50-yawprice3x

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL-INFORMATIVE

**created**: 2026-08-23T00:24:50+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-amp-m2-turnclone-yawcmd-tip50

**wandb_id**: qs91ii4y

**hypothesis**: Plain English: tip50's own verdict found 2M of RL under the unchanged yaw pricing recipe made turning WORSE than the raw untrained BC-turn-clone (eval_yaw err 0.1035 raw -> 0.140 after RL), while training reward rose the whole time -- an 08-21-style misalignment (reward up, target metric down), because yaw income (k_walk_yaw=1.0, k_yaw_prog=1.0) is priced too low relative to translation+style income to protect the newly-available turning skill during fine-tuning. Single lever vs tip50 (continuing from ITS OWN checkpoint, not restarting): triple the two yaw income terms (k_walk_yaw 1.0->3.0, k_yaw_prog 1.0->3.0), everything else byte-identical (same 0.5 turn-in-place exposure, same style weight, same fresh-disc-already-warm state).

**gate**: Discovery continuation (2M, DR-0). eval_yaw on the run's own pod cfg (--speed 0.08 --wz-max 0.3, goal.walk_phase_run_on_yaw=1 included): PASS if turn err <=0.10 (closes the M2-yaw gate outright). INFORMATIVE/repriced-helps if turn err improves toward/below the raw clone's own 0.1035 without regressing translation gait_valid below 10/12. NO-CHANGE/repriced-doesn't-help if err stays ~0.14 or worse despite 3x pricing -- then the bottleneck is not income magnitude (structural lever, e.g. mirror-symmetry regularizer, is next).

**verdict**: REFUTED: tripling yaw income (k_walk_yaw/k_yaw_prog 1.0->3.0) from the tip50 checkpoint made yaw tracking WORSE, not better. eval_yaw (identical cfg to tip50's own eval, run's own pod cfg + the new pricing): turn |wz_err| med 0.208 (was 0.140 at 1x pricing, vs the raw untrained clone's 0.1035) -- every scenario regressed (tip-left/right 0.202/0.264 vs 0.160/0.135; arc 0.18-0.24 vs 0.11-0.15; even fwd-hold zero-wz drift grew 0.079->0.110, hold-gate still under 0.05 cap but rising). Training reward more than doubled (240->564/quarter, walk_yaw reward and yaw_prog reward both pegged near their kernel ceilings 0.99/1.72) while the actual behavioral metric got worse -- confirms this is NOT an income-magnitude problem: simply paying more for the same yaw kernel just lets the policy farm the (now larger) reward via whatever it was already doing (translation+small residual rotation) without improving true wz tracking, an even sharper 08-21 misalignment than the 1x baseline. Translation/gait fully unaffected (walk_speed 0.097, height_err 19mm, direction_valid 0.988, same terminations count as the 1x sibling: 1 tilt_roll/1 tilt_pitch/199 truncated, zero new falls) -- this is a clean, isolated NO-CHANGE-except-worse result, not a destabilization confound. CONCLUSION per the pre-registered gate: 'repriced-doesn't-help' branch fires -- the bottleneck is not yaw income magnitude relative to translation/style; the next lever must be STRUCTURAL (mirror-symmetry regularizer, or a turn-specific gait-phase term/observation), matching this lineage's original fallback plan. Do not spend more budget on yaw-pricing-dose sweeps on this recipe.

