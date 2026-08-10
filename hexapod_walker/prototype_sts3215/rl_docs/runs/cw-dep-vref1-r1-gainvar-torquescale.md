# cw-dep-vref1-r1-gainvar-torquescale

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T18:37:05+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-dep-vref1-r1-gainvar

**hypothesis**: Check that the hardware-candidate walker still walks when its servos are BOTH weaker than expected AND inconsistently tuned at the same time -- the two actuator-strength uncertainties it will actually face on battery power tonight. The gainvar single-axis PASS carried the widest margin of any axis (det slip ~12% over the band edge) with an explicit pre-registered watch-item: 'worth a second look if stacked with another actuator axis'; torquescale (battery-sag 0.5-1.05x) PASSed cleanly solo. This 2-axis compose (2x kp/kv gain spread + widened torque sag) on the contract-exact base cw-dep-vref1-r1 tests whether those margins stack or compound. Prediction-if-true: own-cfg det+sto gv 12/12, 0 term, slip medians within +-20% of vref1-r1's band, fingerprint episodes no worse than the worst PASSed sibling (torquescale, prog 0.47-0.57). Prediction-if-false: weak-AND-mistuned servo draws push beyond the fingerprint into new craters or falls -- flags a real battery-day actuator envelope limit. Strongest alternative: creep/park to dodge weak-servo draws -- check videos, not just scalars. k_current=0 per P0 rule 3 (inherited).

**gate**: own-cfg (DR0.35 + both overrides) det+sto 6/6 @15s: gait_valid 12/12, 0 term, slip/m medians within +-20% of vref1-r1's own band (0.89-1.13 det / 1.13-1.36 sto); lineage fixed-seed fingerprint (det/5, sto/0, sto/1 degraded + sto/4 crater) PRE-ALLOWED as baseline unless worse than torquescale's prog 0.47; DR0 no-override retention clean; frames watched det for flag-leg/skate

