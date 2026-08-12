# cw-dep-bcgait1-hard1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-12T03:09:51+00:00

**pod**: hexapod-mjx-train-1

**steps**: 10000000

**parent**: cw-dep-bcgait1

**wandb_id**: j3qrm13i

**hardware_ready**: False

**hypothesis**: Take the first walker that ever escaped the low-crouch leg-splay habit and polish it with more steps on the identical recipe (BC-INIT tall-gait warm-start + tip1 dep stack + walk_height_gate + DR 0.35 + tipped starts), same one-variable pattern that hardened bc1 into bc1-hard1 and holdbc1 into holdbc1-hard1. Prediction-if-true: probe_tall_wall height stays in the -10..+6mm band (no re-drift toward the old crouch) while slip/m drops toward the <=1.8 bar and the sto sacrificed-leg episode disappears -- budget alone cleans up the rough edges the way it did for rise and hold. Prediction-if-false: height re-drifts back toward -70mm under more steps (the fine-tune eventually unlearns the BC-INIT prior the same way ordinary PPO noise erodes any soft prior) -- meaning BC-INIT needs a live anchor during the whole fine-tune, not just an initialization, and the next lever is bc_init + bc_anchor combined.

**gate**: PASS if probe_tall_wall steady height stays >= -20mm at 10M (no crouch re-drift) AND slip/m det <= 1.8 AND sto gait_valid >= 5/6 (no leg-sacrifice) AND zero falls. FAIL if height re-drifts toward -70mm (BC-INIT prior erodes under budget, same failure class as every warm-start-without-anchor arm) OR slip/leg-sacrifice gets WORSE with steps (budget amplifies the rough edges instead of smoothing them).

**verdict**: PASS per pre-registered gate -- 10M budget cleans up bcgait1's rough edges exactly as predicted, without losing the broken tall-wall. probe_tall_wall steady height -12.7mm mean (seeds -13.0/-12.6/-12.6), inside the >=-20mm no-redrift band (parent 2M: -10..+6mm) -- no crouch relapse. slip/m collapses to med 1.30-1.33 in all four eval slices (det/sto x DR0/DR0.35), well under the 1.8 bar and down from parent's det 2.12 / sto 12.39-with-a-sacrificed-leg; ZERO sacrificed-leg episodes anywhere (24/24 gait_valid True) vs parent's one stochastic leg-sacrifice; ZERO falls in any slice (terms 0 all four reports). Visual quality: roll tail 1.6-2.7deg (DR0) / 2.1-2.4deg (own DR0.35) all four slices, well under the 4deg bar, settled 5/6-6/6; contact sheets show a genuinely tall, six-leg cycling gait (no flag-leg/tripod). One watch item, not gate-relevant: leg-yaw limit margin shrank from parent's +17..+18deg to +1.4..+1.9deg -- still positive (not re-pinned at the 35deg splay limit like every pre-bcgait1 arm) but a real move back toward the old habit; worth tracking on the next continuation.

