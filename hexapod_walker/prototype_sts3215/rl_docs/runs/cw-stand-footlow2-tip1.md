# cw-stand-footlow2-tip1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-12T16:22:56+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-footlow2-hard1

**wandb_id**: febuxkrz

**hardware_ready**: False

**hypothesis**: Teach the deployed-candidate stance checkpoint to correct a body lean during standing, without repeating the mixed-DR mistake that broke it last time. footlow2-hard1 showed a persistent ~8deg lean on its first hardware stand (2/2 round trips, operator bench 08-12), and a same-cycle sim probe (forcing an isolated 8deg tip at hold-episode start, dr-scale 0.0, only dr.tipped_start_prob/deg overridden, no other DR) found the policy ALREADY partially self-corrects -- roll settles to <=2.6deg in 11-12/12 episodes ('recovered'/'settled') -- but often misses the strict final-height/current spec (valid_plant only 5/12 det, 9/12 sto; failures are height_err 15-31mm over spec, one also over-current). This run adds ONLY that one axis to training (dr.tipped_start_prob=0.5, dr.tipped_start_deg=6-10, isolated at dr-scale 0.0, everything else byte-identical to hard1's own recipe) -- unlike the prior attempt (cw-stand-footlow2-level1, which mixed dr-scale 0.35 + ground_tilt 5deg + tipped_start 0.30 in ONE run and reopened the two-foot-park exploit on plain flat-floor retention) this isolates the ONE variable that actually targets the lean, fixing that run's one-variable-per-run violation. Prediction-if-true: forced-8deg-tip det+sto hold valid_plant improves toward the probe's own roll-recovery rate (~11-12/12) via tighter final-height tracking, with zero new park (all six feet duty >=0.5 every episode); nominal untipped retention (hold/rise/lower) stays at hard1's own clean levels. Prediction-if-false: isolating the axis alone still reopens the park (the exploit is generic to ANY additional hold-mode gradient pressure on this BC-anchored lineage, not specific to the DR0.35/ground_tilt confound) -- implicating the anchor itself, not the DR-axis mix, as the blocker.

**gate**: PASS if: forced-8deg-tip probe (dr.tipped_start_prob=1.0, dr.tipped_start_deg=8,8, isolated, dr-scale 0.0, matched vs the frozen hard1 baseline already on file) shows det+sto hold valid_plant >= 9/12 each (vs hard1's own baseline 5/12 det, 9/12 sto) with zero new foot-duty park (all six feet duty >=0.5 across every episode); AND nominal (untipped, dr-scale 0) retention matches hard1's own clean band (hold det+sto 6/6, all six feet duty >=0.9, no park; rise/lower det+sto >=10/12 each, zero flag-leg on video). FAIL if the forced-tip valid_plant rate does not improve over the matched frozen-parent probe, or if nominal retention reopens the two-foot park (any foot duty <0.5 AND end_clear>2mm) -- either FAIL means the anchor itself is implicated, not the DR-axis confound, and no further isolated-DR retry should be queued on this lineage.

**verdict**: FAIL — both pre-registered clauses fire, and the pathology is worth the operator`s eyes: training at 50% tipped spawns taught the policy to LIVE TILTED instead of leveling. Forced-8deg probe vs matched frozen hard1: valid_plant det 12/12 / sto 6/12 (parent 0/12 both — height now held) BUT the body never levels (roll tail med 7.2deg, settled 0/12 vs parent 11/12 at tail <=2.6deg) and every det episode parks a foot (min duty 0.01-0.03 vs gate`s all-six >=0.5). Nominal retention BROKEN: untipped det hold ends tilted 7.6deg with parks, and 6 tilt_roll falls across nominal modes (lower det 1, hold sto 2, lower sto 3) vs parent`s clean 12/12. Per the gate: the anchor is implicated, NOT the DR-axis confound — no further isolated-DR retry on the footlow2 lineage. Tipped-start DR axis on anchored stance: CLOSED (harmful).

