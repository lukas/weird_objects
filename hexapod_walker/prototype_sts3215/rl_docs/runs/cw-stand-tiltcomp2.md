# cw-stand-tiltcomp2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-13T09:19:54+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-tiltcomp1

**wandb_id**: rgjcwp4a

**hardware_ready**: False

**hypothesis**: Teach the standing robot to LEVEL itself after being tipped, by fixing a measured defect in the tilt-comp teacher rather than changing dose or reward. The previous arm's teacher (cw-stand-tiltcomp1) was probe-measured to be mathematically incapable of demonstrating leveling: it commands a correction proportional to the CURRENT lean, so the correction shrinks as the student levels - a perfect student of it settles at 3.95deg from 6.5deg tipped spawns (P-controller fixed point, predicted 3.98deg), above the 3deg gate bar, and hold income was NOT the blocker (leveling tripled income: -0.046 vs -0.150/tick; k_track prices tilt at sigma 1.5deg against a level ref). This arm changes ONE variable: train.bc_anchor_tilt_from_settle=1.0 sources the counter-rotation from the episode's post-settle lean (a per-episode constant), which probe_tilt_teacher verifies makes a perfect student settle at 1.76deg earning +0.385/tick. Prediction-if-true: the forced-8deg-tip probe shows the child settling <=3deg with valid_plant and no parked foot, nominal retention in the parent band. Prediction-if-false: the child again under-adopts the teacher (tail near 6deg, high action-vs-target MSE) - then the blocker is EXPOSURE (hold=0.1 mix / tip prob), not teacher design, and the next lever is the mix, not another teacher. Strongest alternative: anchor supervision at coef 1.0 is too weak against the warm-start habit regardless of teacher correctness.

**gate**: Matched forced-8deg-tip probe (dr-scale 0, dr.tipped_start_prob=1.0 deg=8,8, seed 0, 12 det + 12 sto) vs frozen parent footlow2-hard1: (a) roll tail med <=3deg AND settled/recovered class in >=9/12 per pass (parent baseline: settles 1.45deg in 11/12 via innate recovery; tiltcomp1 failed at 5.75deg 0/24), (b) valid_plant >=9/12 per pass, (c) no parked foot (per-episode min foot duty >=0.5 in det hold), (d) nominal retention: standard eval hold det 6/6, min-duty >=0.9, no falls, rise/lower within parent band. FAIL branch: if tail stays >=5deg with high act-vs-target MSE, next lever is tipped/hold exposure (one variable), teacher design is NOT retried.

**verdict**: FAIL per pre-registered gate clause (a), but the FAIL branch's discriminator answered cleanly: this is under-ADOPTION, not teacher design. Forced-8deg-tip probe (matched protocol, seed 0): det roll tail med 5.25deg (bar <=3), roll_class leaning 24/24 (1 sto fall), parked foot persists (det min duty 0.01-0.05) - marginally better than tiltcomp1's 5.75 but nowhere near the teacher's demonstrated capability. On-pod teacher-capability probe (probe_tilt_teacher, n=6): the settle-lean teacher levels a PERFECT student to 1.76deg at +0.384/tick, while the trained child adopted only ~10% of the correction (tail act-vs-target MSE 0.0131-0.0147 vs total signal size 0.0149) and sits at the tilt-blind income level (-0.143/tick). Root read: 2M steps with hold=0.1 x tipped=0.5 gives ~5% tipped-hold exposure and the hold-tilt correction is ~10% of the anchor loss mass - the supervision never competed with the warm-start habit. Nominal retention (tipped_start_prob=0 rerun): identical to sibling tiltcomp1's band (hold det 6/6 valid_plant, tail 0.35deg, zero falls, min-duty 0.72/slip 0.632 vs hard1's pristine 0.95/0.136 - pre-existing lineage cost, NOT new). Honest open point: the untrained parent's innate recovery (1.45deg in 11/12) still beats every tipped-trained child - tipped-spawn training erodes recovery under all three teacher designs; whether exposure fixes adoption or the erosion is training-dynamics (e.g. trip-fear) is what the pre-registered exposure arm cw-stand-tiltcomp3 (hold=0.4,rise=0.3,lower=0.3, one knob) discriminates. hardware-ready: no (visible persistent lean on video).

