# cw-dep-vref1-r1-placement-comshift

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T17:05:11+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: nbaoe33l

**hardware_ready**: False

**hypothesis**: Plain English: test whether the hardware candidate still walks cleanly with two realistic MECHANICAL quirks together instead of one at a time -- imprecise joint placement/assembly (6deg) AND an off-center battery/harness CoM (0.03m), which the real chassis will have simultaneously. vref1-r1 already PASSED joint-placement noise and CoM offset INDIVIDUALLY tonight; this bundles them onto the same base recipe as its siblings (respec of cw-dep-vref1-r1, not warm-started off either single-axis checkpoint, to avoid compounding one lineage's drift). Per P0 rule 3, k_current=0. If-true: own-cfg (DR0.35 + both axes) det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- the two individually-benign mechanical axes stay benign combined, same pattern as comshift+deadband and fric+groundtilt5. If-false: combined mechanical error (assembly tolerance stacked on CoM shift) breaks tracking in a way neither axis did alone -- flag as a real pre-attempt-#2 assembly risk.

**gate**: own-cfg (DR0.35 + dr.placement_noise_deg=6.0 + dr.com_offset_m=0.03) det+sto 6/6 @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1 own band; DR0 retention clean; frames watched det

**verdict**: PASS (if-true): the two individually-benign mechanical axes (6deg joint-placement/assembly noise + 0.03m off-center CoM) stay benign COMBINED, same pattern as comshift+deadband and fric+groundtilt5. DR0-gate own-cfg det+sto: det gv 6/6, 0 term, slip/m median 1.28 (sorted [0.96,1.00,1.14,1.41,1.46,29.67], last is the lineage's known fixed-draw march-in-place crater at idx4, video-confirmed no flag-leg/drag) -- ~13% over vref1-r1's own 1.13 det ceiling, inside the +-20% tolerance (matches gainvar's precedent as the widest-margin single axis); sto gv 6/6, 0 term, slip/m med ~0.97, in band. Video (det ep0 clean six-leg gait, det ep4 crater) matches every prior PASSed sibling compose. Second real assembly-tolerance-stack axis (after comshift+deadband) clears clean.

