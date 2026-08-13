# cw-quadwalk4

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-13T15:48:44+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-quadwalk3

**wandb_id**: rnqq6ouh

**hypothesis**: Start every four-leg-walking episode with the front legs ALREADY lifted, so the robot only has to learn to step with the rear four — not to discover lifting in the first place. Three arms (cw-quadwalk1/2/3) proved pricing cannot do it: even a live per-tick fine worth ~40% of episode return (quadwalk3, charge verified fired at -575/ep) would not pull a warm-started six-leg walker out of its basin from the all-six-feet plant start, while the same policy lifts its fronts fine when told to hold still (quad mode). This arm keeps quadwalk3's exact economy (3x lift income + k_quad_lift_contact=3.0) and changes ONE thing, the new spawn package goal.quadwalk_start=quad: episodes begin in the statically-viable four-leg stance (mid feet splayed, fronts at the feasibility tuck claw), tilt refs anchored level, safety envelope widened to 25deg to cover the measured ~16deg limp-settle sag transient (part of the spawn contract, like getup any-starts). Now six-leg walking requires actively PLANTING the charged fronts, so the priced-out cheat is no longer the path of least action. Prediction-if-true: det video by 2M shows the fronts staying up past the grace window (tail lift duty <0.15) with genuine rear-four stepping attempts and positive forward translation. Prediction-if-false: the policy plants the fronts right after spawn and six-leg walks anyway (same one-line STOP - would mean the basin is reachable and preferred even from outside, i.e. the task needs a stronger curriculum or BC-style reference), or it freezes/creeps in the spawn stance (progress income unearned - exploration blocked even with the stance given), or it walks while staying pitched near the sag (attitude terms underpriced vs walk income - a NEW finding, not a rerun).

**gate**: Harness quadwalk det 6 eps @2M with the run's own cfg: >=4/6 eps net forward displacement >= +0.05m AND fronts lifted (post-grace tail lift duty <0.15 both lift legs) AND no episode net backward < -0.02m AND 0 falls AND roll/pitch settled near level at episode end (roll_tail <= 3deg med; not parked at the ~16deg spawn sag). Det video shows all four support legs cycling contact/swing. Retention: quad-hold survived_frac 1.0, fronts lifted. Known cheats = one-line STOP, no continuation: fronts planted post-spawn six-leg gait, freeze/creep-in-stance, backward shuffle, walking-while-sagged. PASS does NOT make it the bank reference (needs full QUADWALK_REF_GATE.md).

