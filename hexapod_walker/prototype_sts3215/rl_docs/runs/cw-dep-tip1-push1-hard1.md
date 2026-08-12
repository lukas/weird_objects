# cw-dep-tip1-push1-hard1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-12T03:51:45+00:00

**pod**: hexapod-mjx-train-3

**steps**: 10000000

**parent**: cw-dep-tip1-push1

**wandb_id**: mi3dbc1l

**hardware_ready**: False

**hypothesis**: Give the torque-shove (walk_push) training recipe the budget it was still using at the 2M discovery buzzer, on the identical one-variable recipe (warm from tip1-push1 itself, same dr.walk_push_prob=0.5/nm=2.0-3.0/s=0.8-1.5, same dep-line stack) -- the same pattern that turned cw-dep-bcgait1's partial existence-proof into cw-dep-bcgait1-hard1's decisive PASS. Prediction-if-true: the matched-parent probe_walk_push.py fall-rate gap widens to >=2x (the pre-registered bar the discovery run just missed at 1.8x) with nominal DR0 retention unchanged. Prediction-if-false: the gap stays flat or narrows under more steps -- meaning the 2M signal was budget noise, not a strengthening mechanism, and the torque-DR family closes for real this time (3rd-and-final arm).

**gate**: PASS if matched-parent probe_walk_push.py (n>=12 seeds/side, same forced 2.6Nm/1.5s dose) shows child fall rate >=2x lower than frozen tip1 AND nominal DR0 walk retention (gait_valid, slip/m, prog_ratio) matches this run's own 2M discovery band with zero new falls. FAIL if the fall-rate gap does not widen past 1.8x (flat or worse) -- torque-DR family CLOSED for good, remaining lever is contact/pinning modeling only.

**verdict**: FAIL -- matched-parent probe_walk_push.py (n=12 seeds/side, forced 2.6Nm/1.5s) gives hard1 (10M) the IDENTICAL fall count as the 2M discovery parent: 5/12=0.417 vs frozen tip1 9/12=0.75, still 1.8x short of the required >=2x bar, and tail-roll among survivors is slightly worse (1.63 vs 1.26 deg median) -- 10M extra steps bought nothing on this metric, bit-for-bit the same discordant seeds (5/8/9/10) as before. Nominal DR0 retention holds: det gait_valid 5/6 (same pre-existing seed-5 sac[3,5] fixed-seed fingerprint as the parent, not new), sto 6/6, slip/m med 1.21 (parent's own 0.96-1.12 band, no falls, terms 0 both passes). Video: clean six-leg crouch gait, no new cheat. Per pre-registration this is the FAIL branch: the torque-DR (walk_push) family is CLOSED FOR GOOD -- budget does not widen a near-miss separation. Remaining lever for the hardware takeoff-roll transient is contact/pinning geometry modeling (belly/tucked-leg collision), not any more DR variant.

