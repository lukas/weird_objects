# cw-dep-vref1-r1-gainvar-torquescale

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: EVALUATED

**created**: 2026-08-10T18:37:05+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-dep-vref1-r1-gainvar

**wandb_id**: axahh2hg

**hardware_ready**: False

**hypothesis**: Check that the hardware-candidate walker still walks when its servos are BOTH weaker than expected AND inconsistently tuned at the same time -- the two actuator-strength uncertainties it will actually face on battery power tonight. The gainvar single-axis PASS carried the widest margin of any axis (det slip ~12% over the band edge) with an explicit pre-registered watch-item: 'worth a second look if stacked with another actuator axis'; torquescale (battery-sag 0.5-1.05x) PASSed cleanly solo. This 2-axis compose (2x kp/kv gain spread + widened torque sag) on the contract-exact base cw-dep-vref1-r1 tests whether those margins stack or compound. Prediction-if-true: own-cfg det+sto gv 12/12, 0 term, slip medians within +-20% of vref1-r1's band, fingerprint episodes no worse than the worst PASSed sibling (torquescale, prog 0.47-0.57). Prediction-if-false: weak-AND-mistuned servo draws push beyond the fingerprint into new craters or falls -- flags a real battery-day actuator envelope limit. Strongest alternative: creep/park to dodge weak-servo draws -- check videos, not just scalars. k_current=0 per P0 rule 3 (inherited).

**gate**: own-cfg (DR0.35 + both overrides) det+sto 6/6 @15s: gait_valid 12/12, 0 term, slip/m medians within +-20% of vref1-r1's own band (0.89-1.13 det / 1.13-1.36 sto); lineage fixed-seed fingerprint (det/5, sto/0, sto/1 degraded + sto/4 crater) PRE-ALLOWED as baseline unless worse than torquescale's prog 0.47; DR0 no-override retention clean; frames watched det for flag-leg/skate

**verdict**: PASS -- stacking BOTH actuator-gain-spread (2x kp/kv, widest single-axis margin so far) AND widened battery-sag (0.5-1.05x) does NOT push the hardware candidate past a real limit. Own-cfg (DR0.35+both) det/sto gv 6/6 each, 0 term, prog med 1.02/0.88, slip med 1.28/1.23 (within +-20% of vref1-r1's own 0.89-1.13/1.13-1.36 band). DR0 retention det/sto gv 6/6, 0 term, prog med 0.92/1.00, slip med 1.25/1.06 (in-band). Degraded episodes (det/4,5 gate; det/5,sto/0,1 own-cfg) exactly match the pre-registered lineage fixed-seed fingerprint -- video-checked (det_0/4, sto_0/4 own-cfg): level body, all six legs cycling every frame, no flag-leg/drag/fall, march-in-place not new pathology. The pre-registered watch-item (gainvar's widest margin, stacked with torquescale) is resolved: margins do NOT compound. Not independently hardware-ready (inherits vref1-r1's paddle-gait economics); 12th protected axis-pair on this line.

