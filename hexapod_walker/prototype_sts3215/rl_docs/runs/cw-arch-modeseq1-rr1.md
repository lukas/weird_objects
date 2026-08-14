# cw-arch-modeseq1-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-14T13:03:14+00:00

**pod**: hexapod-mjx-train-0

**steps**: 10000000

**parent**: cw-arch-gru-dual2

**hypothesis**: Teach one model to stand up, walk, sit down and stand up again on command, in a single continuous run - this arm tests whether training directly on chained mode sequences (the new goal.mode_seq episode generator, 75% sequence / 25% single-mode diet) gets the RL policy through the mode switches that specialists only survive when an external script re-anchors them. TRANSITIONS_DIRECTIVE Arm 2: dual1's proven dual-core GRU stack, warm from the dagger1 BC init per the pre-registered warm-start order (transdagger2 gate FAIL on the two rise clauses, gru-dual2 gate FAIL), mode sequencing as the ONLY new variable.

**gate**: Pre-registered (TRANSITIONS_DIRECTIVE Arm 2): sequence eval det+sto DR0 + own-DR0.5 zero falls >=11/12 det AND per-segment criteria >=9/12 AND single-mode retention at dual1 levels (walk gait_valid >=5/6 prog >=0.80, hold >=4/6, lower >=4/6, rise n=12 method >= its own init 3/12) AND switch-window max tilt reported (baseline, no bar in v1).

**refused_reason**: hexapod-mjx-train-0 code marker 7cda50de8cd13f797dc4b0c02da4725a82cb932b-dirty != local HEAD 7cda50de8cd13f797dc4b0c02da4725a82cb932b. Sync first: snapshot.sh --sync hexapod-mjx-train-0 (and snapshot/commit before that if the tree is dirty).

