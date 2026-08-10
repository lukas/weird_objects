# cw-dep-vref1-r1-badstart

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T19:02:10+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**hardware_ready**: False

**hypothesis**: Plain English: test whether the hardware candidate still walks cleanly when the human placing the robot before an episode gets a joint noticeably wrong more often (higher PROBABILITY of a bad start), separate from today's failed start-variation compose which changed several things at once including a brand-new, unvetted zero-drift-frame mechanism. If-true: own-cfg (DR0.35 + dr.bad_start_prob=0.5, double the nominal 0.25) det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- elevated bad-start probability ALONE is not the startvar1 compose's problem. If-false: bad-start probability alone (with no zero-drift-frame) already breaks gait_valid or inflates slip -- contradicts today's noBS1 isolation finding that bad_start_prob only partially explained startvar1's failure, and reframes the priority for the zero-drift-frame mechanism rework.

**gate**: own-cfg (DR0.35 + dr.bad_start_prob=0.5) det+sto @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (det ~0.89-1.13, sto ~1.13-1.36) +-20%; DR0 retention clean; frames watched det; the lineage's known fixed-draw crater (det/4 or det/5) is pre-allowed as baseline

**verdict**: PASS -- doubling bad-start joint-placement PROBABILITY (0.25->0.5 of the nominal, tests frequency not magnitude/breadth) composes free onto the hardware candidate. DR0-gate (dr-scale 0.0, only bad_start_prob=0.5 applied): gait_valid 12/12, 0 term, slip/m med det 1.08/sto 0.97 (within vref1-r1own 0.89-1.13/1.13-1.36 band); the lineages known fixed-draw crater (det/4) plus TWO additional degraded stochastic episodes (sto/3, sto/5, prog 0.42-0.56, slip 2.4-4.0) -- more instances of the SAME march-in-place stall than usual (typically 1/12), a mechanically expected consequence of doubling the probability (fixed eval seed -> more of the 12 draws now cross the higher threshold), not a new pathology (video-checked all three: level body, six legs still cycling, no flag-leg/drag/fall). own-cfg DR0.35 (matching training distribution) pass is CLEANER: gait_valid 12/12, 0 term, slip/m med det 1.09/sto 1.03 (in-band), with only MILD degradation (det/3 slip1.44, sto/0 slip1.61) and no full crater at all -- better than most DR0.35 siblings tonight. Confirms if-true: elevated bad-start probability alone is absorbed; gait_valid never breaks, no terminations, no new pathology at 2x nominal frequency.

**refused_reason**: hexapod-mjx-train-2 already runs cw-dep-vref1-r1-badstart — GPU pods host exactly one run; pick a free GPU pod.

