# cw-dep-vref1-r1-torquescale-deadband

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T17:49:08+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**hypothesis**: Plain English: test whether the hardware candidate still walks cleanly with two realistic ACTUATOR quirks together instead of one at a time -- weaker available torque (battery sag under load) AND servo dead-zone response, which the real STS3215s will show simultaneously when the battery is low and the joints are worn. vref1-r1 already PASSED widened torque-sag (0.5-1.05x) and servo deadband (1-3x) INDIVIDUALLY tonight; this bundles them onto the same base recipe as its siblings (respec of cw-dep-vref1-r1, not warm-started off either single-axis checkpoint, to avoid compounding one lineage's drift). Per P0 rule 3, k_current=0. If-true: own-cfg (DR0.35 + both axes) det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- the two individually-benign actuator axes stay benign combined, same pattern as comshift+deadband and fric+groundtilt5. If-false: reduced torque headroom combined with a dead-zone response compounds worse than either alone (less margin to punch through the dead-zone) -- flag as a real pre-attempt-#2 actuator risk, directly relevant to today's loaded-actuator finding (P0 rule 6).

**gate**: own-cfg (DR0.35 + dr.torque_scale=0.5,1.05 + dr.deadband_scale=1.0,3.0) det+sto 6/6 @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band; DR0 retention clean; frames watched det

