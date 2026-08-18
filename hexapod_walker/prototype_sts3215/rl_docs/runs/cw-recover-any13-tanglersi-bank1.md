# cw-recover-any13-tanglersi-bank1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-16T18:47:27+00:00

**pod**: hexapod-mjx-train-0

**steps**: 30000000

**parent**: cw-recover-any11-rsi-scratch1

**wandb_id**: xpqpbumg

**hardware_ready**: False

**hypothesis**: Teach the fallen robot's hardest get-up case (legs tangled) by letting it practice starting from partway through its OWN past successful untanglings, instead of always the raw tangled pose. any7/any11/any12 all plateaued on the tangle-family kinds at the same statistically stable ~0.25-0.44 CERT success band under three different curriculum-weight settings, closing that lever; this is the pre-registered next mechanism (a harvested on-path RSI bank, generalizing the belly->plant RSI trick that already solved the zero-bucket wall, to a family with no single hand-built reference trajectory). Warm from any11 (NOT any12, which regressed on basic-bucket retention), default curriculum mix restored (any12's 0.80 focus concentration is now closed as actively harmful), zero-family ref-path RSI kept on for zero-safety. Bank: harvested this cycle from any11's own DETERMINISTIC rollouts on tangle_mild/tangle_mid/tangle/tangle_deep (350 episodes/kind; success rates 0.926/0.497/0.377/0.377, matching the historical band and confirming the source checkpoint is a real, if partial, teacher). Prediction-if-true: once bucket 15 (tangle+bank) becomes frontier, its CERT success_fraction climbs above the 0.25-0.44 band and sustains >=0.7 across >=2 late certs, while buckets 0-10 and the zero bucket hold their >=0.8 retention floor (no repeat of any12's starvation). Prediction-if-false: the bank's on-path states aren't actually informative (e.g. because they're states the policy already frequently passes through anyway) and tangle stays stuck in the same band even with harvested-bank practice -- closing curriculum/exposure-side fixes for tangle entirely and pointing at a reward- or BC-teacher-side redesign instead.

**gate**: Read at 30M or earlier plateau: 'tangle' kind CERT success_fraction (16-ep denominator, the actual wall kind paired with 'bank' at bucket 15) must sustain >=0.7 across >=2 consecutive late certs once it becomes frontier -- that is a WIN for the harvested-bank RSI lever. FAIL (plateaus in the pre-existing 0.25-0.44 band or below, or frontier never legitimately reaches B15) closes on-path-bank RSI for tangle and calls for a reward/BC-teacher-side redesign, not another exposure mechanism. Zero-bucket (RSI-protected) AND buckets 0-10 retention must stay >=0.8 at the final cert -- a regression there would mean the new bank-RSI axis repeats any12's starvation mistake even with the default curriculum mix restored. video-verify any earned tangle frontier (no flag/stilt/park) before crediting improvement.

**verdict**: FAIL on both pre-registered conditions. Tangle CERT success_fraction never sustained >=0.7: it touched 0.75 twice in a row (24.1M/25.0M) but decayed to a 0.1875 trough at 28M and closed the 30M budget at 0.5, well under the win bar (win requires sustaining, not briefly touching). Independently, the retention floor the gate was protecting broke: the zero bucket (RSI-protected) read 1.0 four times through 24M then crashed to 0/16 at its final reading (29M), and bucket 10 crashed to 0/16 at BOTH of its last two readings (28M, 29M) right after reading 1.0 at 23-24M -- the same starvation signature any12 showed at 0.80 focus concentration, now appearing even with the DEFAULT curriculum mix restored (n=16 per reading, so 0/16 after a run of 1.0s is a real regression, not sampling noise). Per the pre-registered gate this CLOSES on-path-bank RSI for tangle (2nd new-mechanism miss on tangle, after 3 curriculum-weight misses any7/any11/any12) and escalates to a reward/BC-teacher-side redesign, which the gate text itself flags as an operator call. Root cause note for that redesign: the recover BC anchor is eligibility-gated OFF whenever the robot is not already near-upright/near-plant (08-15 anchor directive, by design), so it structurally cannot supervise the tangled-to-upright transition itself -- a real fix needs a tangle-specific reference or a relaxed eligibility gate, a design choice not another automatic exposure knob. any11 stays the recovery line's reference checkpoint (independently corroborated by MCP note fb_20260816T203228_bc9bad). No tangle-kind episode was captured in this run's periodic video reels (all logged reels landed on plant_catch/onefoot/crouch/partial kinds), so this verdict rests on the CERT/W&B numeric record, which is unambiguous on its own for a FAIL.

