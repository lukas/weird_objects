# cw-dep-vref1-r1-placement-comshift

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-10T17:04:15+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**hypothesis**: Plain English: test whether the hardware candidate still walks cleanly with two realistic MECHANICAL quirks together instead of one at a time -- imprecise joint placement/assembly (6deg) AND an off-center battery/harness CoM (0.03m), which the real chassis will have simultaneously. vref1-r1 already PASSED joint-placement noise and CoM offset INDIVIDUALLY tonight; this bundles them onto the same base recipe as its siblings (respec of cw-dep-vref1-r1, not warm-started off either single-axis checkpoint, to avoid compounding one lineage's drift). Per P0 rule 3, k_current=0. If-true: own-cfg (DR0.35 + both axes) det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- the two individually-benign mechanical axes stay benign combined, same pattern as comshift+deadband and fric+groundtilt5. If-false: combined mechanical error (assembly tolerance stacked on CoM shift) breaks tracking in a way neither axis did alone -- flag as a real pre-attempt-#2 assembly risk.

**gate**: own-cfg (DR0.35 + dr.placement_noise_deg=6.0 + dr.com_offset_m=0.03) det+sto 6/6 @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1 own band; DR0 retention clean; frames watched det

**refused_reason**: hexapod-mjx-train-4 already runs cw-dep-vref1-r1-imupos — GPU pods host exactly one run; pick a free GPU pod.

