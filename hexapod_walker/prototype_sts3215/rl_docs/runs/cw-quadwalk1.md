# cw-quadwalk1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-13T13:05:11+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-quad-hold2

**wandb_id**: c40l7m47

**hardware_ready**: False

**hypothesis**: Can the robot learn to WALK on its four rear legs with the front pair held up as hands? Scripted four-leg gaits are measured geometrically impossible on this body (the balance point can never sit far enough back), so this arm tests whether closed-loop RL - which can shift weight dynamically tick by tick - discovers genuine rear-four stepping where open-loop scripts cannot. Warm from the four-leg-stance champion cw-quad-hold2 (already balances on four with fronts lifted); ONE lever: the goal mix swaps walk/hold for the new quadwalk mode (0.7, quad-hold 0.3 guards the stance) plus the bank-proven k_quad_still=1.0 pricing stance creep. Prediction-if-true: by 2M the det video shows all four support legs cycling contact/swing with positive forward travel and fronts lifted. Prediction-if-false (failure class picks the next lever): (a) freeze-in-stance => quad plant/clear income out-earns slow-band walk progress, reprice; (b) fronts-down six-leg gait => lift-leg exemptions/clear income too weak; (c) genuine stepping attempts but falls => balance curriculum (slower band, longer grace). Strongest alternative: dynamic balance on this geometry is beyond a reactive MLP at any reward - would show as (c) persisting across curricula. First arm under the 08-13 operator ruling (route 2); known cheats (freeze, fronts-down walk, backward-rectified shuffle) are pre-registered one-line STOPs; passing THIS discovery gate does NOT make it the bank reference - that requires rl_docs/tracks/quad/QUADWALK_REF_GATE.md.

**gate**: Harness quadwalk det 6 eps @2M: >=4/6 eps net forward displacement >= +0.05 m AND fronts lifted (post-grace tail lift duty <0.15 both lift legs) AND no episode net backward < -0.02 m AND 0 falls; det video/contact-sheet shows all four support legs cycling contact/swing (no pinned/dragged mid leg, no outrigger). Retention: quad-hold mode survived_frac 1.0 with fronts lifted and planar creep <= 0.10 m/15 s (first k_quad_still use). Any known cheat dominating video (freeze, fronts-down gait, backward shuffle) = STOP, no continuation.

**verdict**: STOP — reward/eval specification bug: pre-registered cheat (b), fronts-down six-leg gait. Det/sto quadwalk gait_valid 0/6, fronts_lifted 0/6 both passes; frame strip shows all six legs planted/cycling on the ground like the ordinary walk gait, no lift attempt at all. Positive net forward (med 0.64m, no backward rectification) and 0 falls, but fronts-down is an automatic FAIL regardless — matches the discovery gate own pre-registered STOP list. quad (hold) retention clean (survived 6/6, roll settled 6/6). No continuation of this exact spec: the lift-leg exemption/clear income is too weak relative to walk forward-progress income to induce lifting; next lever is reward-side per the pre-registered failure-class map (b), not more steps.

