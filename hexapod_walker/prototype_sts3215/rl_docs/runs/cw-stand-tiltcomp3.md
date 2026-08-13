# cw-stand-tiltcomp3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-13T09:52:55+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-tiltcomp2

**wandb_id**: hn9zchh3

**hypothesis**: Give the standing robot enough LESSONS in leveling for the teaching to stick: the previous arm proved the tilt-correction teacher is capable (a perfect student levels to 1.76deg and triples its reward) but the trained policy adopted only ~10% of the correction — leveling supervision was ~5% of training episodes (hold=0.1 x tipped=0.5) and a rounding error in the anchor loss next to rise/lower. This arm changes ONE knob from cw-stand-tiltcomp2: goal mix hold 0.1->0.4 (rise/lower 0.45->0.3 each), quadrupling hold/tipped-hold exposure with the identical probe-verified teacher. Prediction-if-true: adoption jumps (act-vs-target MSE < 0.5x the signal size) and the forced-tip probe settles <=3deg. Prediction-if-false: adoption stays ~0 despite 4x exposure — then tipped-spawn training itself erodes recovery (the untrained parent still recovers to 1.45deg, better than every tipped-trained child) and the route escalates to the operator WITH the full mechanism dossier (capable teacher + paying income + refused adoption = training-dynamics problem, e.g. trip-fear near the 10deg roll limit). Strongest alternative: exposure helps adoption but rise/lower retention pays for it (rise 0.45->0.3 exposure cut) — caught by the retention clause.

**gate**: Forced-8deg-tip probe (dr-scale 0, dr.tipped_start_prob=1.0 deg=8,8, seed 0, 12 det + 12 sto, matched protocol): (a) det roll tail med <=3deg with settled/recovered >=9/12 (tiltcomp2 baseline: 5.25deg, 0/24; parent hard1: 1.45deg 11/12), (b) adoption: probe_tilt_teacher policy arm tail act-vs-target MSE < 0.5x mse(qnom,tgt) (tiltcomp2: ~0.9x), (c) no parked foot: det min foot duty >=0.5, (d) retention (tipped_start_prob=0 rerun): hold det 6/6 valid_plant tail <=1deg zero falls, rise det+sto and lower within the tiltcomp1/2 sibling band (rise det vp >=5/6), drag/slip not worse than sibling band. FAIL branch: adoption still ~0 => tipped-exposure training is CLOSED for real this time (teacher-capability and income both measured innocent) and the standing-lean route goes to the operator with the dossier; adoption up but retention broken => next knob is anchor coef balance, not mix.

