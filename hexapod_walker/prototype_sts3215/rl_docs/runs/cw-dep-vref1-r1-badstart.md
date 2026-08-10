# cw-dep-vref1-r1-badstart

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-10T18:58:26+00:00

**pod**: hexapod-mjx-train-6

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**hypothesis**: Plain English: test whether the hardware candidate still walks cleanly when the human placing the robot before an episode gets a joint noticeably wrong more often (higher PROBABILITY of a bad start), separate from today's failed start-variation compose which changed several things at once including a brand-new, unvetted zero-drift-frame mechanism. If-true: own-cfg (DR0.35 + dr.bad_start_prob=0.5, double the nominal 0.25) det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- elevated bad-start probability ALONE is not the startvar1 compose's problem. If-false: bad-start probability alone (with no zero-drift-frame) already breaks gait_valid or inflates slip -- contradicts today's noBS1 isolation finding that bad_start_prob only partially explained startvar1's failure, and reframes the priority for the zero-drift-frame mechanism rework.

**gate**: own-cfg (DR0.35 + dr.bad_start_prob=0.5) det+sto @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (det ~0.89-1.13, sto ~1.13-1.36) +-20%; DR0 retention clean; frames watched det; the lineage's known fixed-draw crater (det/4) is pre-allowed as baseline

**refused_reason**: hexapod-mjx-train-6 code marker b9f767f42b96c90e222d4fde6e8241066f99bfd3 != local HEAD 95746fde1cee9ed57409c83975d7bc3ba76f9da4. Sync first: snapshot.sh --sync hexapod-mjx-train-6 (and snapshot/commit before that if the tree is dirty).

