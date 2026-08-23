# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-seed23

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-23T10:36:02+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

**wandb_id**: ojshhjgv

**hypothesis**: Seed-robustness twin of this cycle's tipfrac05-pushcal518 arm (recalibrated dr.ext_push_n=5-18N on the full turn+fault+push composition): the earlier (uncalibrated) 7-seed batch found near-universal (5/6, then 6/7) det/3-style falls across seeds, so a single seed=7 PASS here would not be enough evidence the recalibration generalizes -- batching this seed alongside seed=7 and seed=13 answers seed-robustness in the same cycle instead of the 4-cycle serial seed-farming the operator flagged 08-22.

**gate**: Same as tipfrac05-pushcal518: own-cfg DR-0 gate, RAW terminated field. PASS = 0/12 real falls. Compare directly against seed7's own result and the original (uncalibrated) seed23 result (2 falls) from the earlier batch.

**verdict**: Seed-robustness twin PASSES: recalibrated push range (dr.ext_push_n 5-18N, down from 10-25N) transfers to seed=23 on the full turn+fault+push composition. Evidence: raw per-episode 'terminated' field (not gait_valid) is False on all 12/12 own-cfg DR-0 episodes (6 det + 6 sto), roll_peak_deg max 14.9 (det/0), no sacrificed-leg episode beyond the benign walk/sto/4 (gait_valid False but terminated=False, same pattern as the seed7 parent). This is the same clean profile as the already-PASSed seed7/pushcal518 parent (also 0/12, same walk/sto/4 sac[0] signature) -- prog/slip medians nearly identical (det prog med 0.90 vs 0.88, slip med 4.22 vs 4.66; sto prog med 0.56 vs 0.55, slip med 5.97 vs 7.08). Contrast: the earlier uncalibrated seed23 arm (10-25N range) fell 2/12 at this same composition. Contact sheet: upright, six legs cycling, no visible pathology. Why: the fault+push+turn fall risk is push-magnitude driven (root-caused this cycle-family to push-disturbance recovery, not turn-in-place or fault), so narrowing the push range fixes it independent of seed. What's next: with seed7 and seed23 both clean, only seed13 (still training, another cycle's) remains to close the pre-registered 3-arm batch; if it also passes, promote pushcal518 as the new M5-candidate safety base for this lineage and retire the old 10-25N push range on turnfault-seq1 descendants.

