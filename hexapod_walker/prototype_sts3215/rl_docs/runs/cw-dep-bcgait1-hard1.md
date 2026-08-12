# cw-dep-bcgait1-hard1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-12T03:09:51+00:00

**pod**: hexapod-mjx-train-1

**steps**: 10000000

**parent**: cw-dep-bcgait1

**hypothesis**: Take the first walker that ever escaped the low-crouch leg-splay habit and polish it with more steps on the identical recipe (BC-INIT tall-gait warm-start + tip1 dep stack + walk_height_gate + DR 0.35 + tipped starts), same one-variable pattern that hardened bc1 into bc1-hard1 and holdbc1 into holdbc1-hard1. Prediction-if-true: probe_tall_wall height stays in the -10..+6mm band (no re-drift toward the old crouch) while slip/m drops toward the <=1.8 bar and the sto sacrificed-leg episode disappears -- budget alone cleans up the rough edges the way it did for rise and hold. Prediction-if-false: height re-drifts back toward -70mm under more steps (the fine-tune eventually unlearns the BC-INIT prior the same way ordinary PPO noise erodes any soft prior) -- meaning BC-INIT needs a live anchor during the whole fine-tune, not just an initialization, and the next lever is bc_init + bc_anchor combined.

**gate**: PASS if probe_tall_wall steady height stays >= -20mm at 10M (no crouch re-drift) AND slip/m det <= 1.8 AND sto gait_valid >= 5/6 (no leg-sacrifice) AND zero falls. FAIL if height re-drifts toward -70mm (BC-INIT prior erodes under budget, same failure class as every warm-start-without-anchor arm) OR slip/leg-sacrifice gets WORSE with steps (budget amplifies the rough edges instead of smoothing them).

