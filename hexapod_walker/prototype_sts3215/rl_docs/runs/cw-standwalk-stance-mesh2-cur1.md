# cw-standwalk-stance-mesh2-cur1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-25T05:55:57+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-standwalk-stance-mesh1-rr1

**hypothesis**: Plain English: the first mesh stance batch failed because grinding into the floor with maxed-out servos was PROFITABLE (+66 to +187/episode measured); this rerun adds a measured price on near-saturation current and on dying early, so the only profitable strategy left is actually standing up, holding, and lowering. Same footlow2-style mesh/100Hz joint_goal recipe as cw-standwalk-stance-mesh1-rr1 (seed 0 of 3), plus the probe-chosen pricing: reward.k_current_hot=1.0 @ current_hot_a=2.0 (quadratic charge only in the 2.0-3.0 A near-saturation band; trip is 2.5 A/0.8 s, ceiling 2.64 A) + term_cost_per_remaining_s=3 capped 60 (bank-then-die pays back its horizon). Dose evidence (probe_stance_pricing on the ACTUAL failed rr1 checkpoint, launch-exact stack, logs/probe_stance_pricing_rr1_hotA.json): grind flipped negative in all modes (hold +66->-39, lower +187->-27, rise-grind -773) while honest keeps dominance (rise replay 2160 = 90% of unpriced, hold quiet 1472 untouched, orderings replay > partial 412 > flagleg 58 > freeze -637 > stilt -771 preserved; flat 1.0 A threshold was rejected because honest mesh mid-crouches legitimately run ~2.2-2.6 A and it sank honest below cheats). Prediction-if-true: per-mode survived_frac does NOT collapse to 0 by 10M (mesh1 signature), rise ends valid plant on the panel majority by 20M. Prediction-if-false: (i) rise/hold recover but lower stays dead -> the 25-55mm crouch is intrinsically too hot on the 3.5kg body; next fork is goal.lower_height_mm mesh recalibration (belly-rest supported), NOT more pricing; (ii) all modes sink toward freeze-conservatism (freeze is -637, so paid refusal is unlikely) -> pricing overdosed, halve k. Strongest alternative: 20M@DR0.2 remains under-budget for from-scratch mesh stance regardless of pricing (the original recipe used a warm-started DR ladder).

**gate**: Stage-1 pre-gate read at 20M, joint across the 3 cur1 seeds: pod_eval stance panel (rise/hold/lower) n>=12 det+sto at DR-0 + own-DR(0.2): zero falls/tips; rise ends valid plant (PLANT_SPEC, height within +-15mm of [79,87]) >=5/6 det AND sto; lower posture-strict >=5/6; hold quiet 6/6, no crash-lowering on video. 2-3/3 healthy = recipe robust -> record mesh reference band; 0-1/3 = escalate the named fork (lower-height recalibration or budget ladder), not more seeds.

