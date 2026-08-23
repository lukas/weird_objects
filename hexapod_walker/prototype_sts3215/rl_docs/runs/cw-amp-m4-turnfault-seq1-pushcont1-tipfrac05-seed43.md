# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-seed43

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-23T09:30:38+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-s2

**wandb_id**: mfidly4j

**hypothesis**: Plain English: is the tipfrac05 turn-in-place recipe's SAFETY 1-in-3-unsafe-seed-basin rate (seed13/s3 fell + sacrificed 3 legs even hazard-free, while seed7/seed23 were clean) a real ~33% failure rate or a small-n fluke? Same exact recipe -- only the RNG seed changes; this arm adds seed=43 (batch sibling of seed31/37/41).

**gate**: Read as a 4-seed batch with seed31/37/41. SAFE = own-cfg DR-0 gait_valid >=11/12, zero falls, <=1 sacrificed leg. UNSAFE = any fall or >=2 sacrificed legs on hazard-free walk episodes. Tip-tracking is a secondary read. Batch verdict: count SAFE/UNSAFE across all 4 (+3 already-run) to pin the true rate.

**verdict**: Seed-safety-variance batch's 4th and last arm (seed31/37/41/43): SAFE on its own — own-cfg DR-0 gate (12 hazard-free-by-config-noise walk episodes, hazards still baked in per training cfg) shows ZERO real falls (raw terminated field checked per-episode, not just gait_valid), gait_valid 11/12 (6/6 det + 5/6 sto), 1 sacrificed leg (walk/sto/4, a wobble-and-recover per video, not a topple). Matches seed7/seed23's prior 'clean' read on the surface. BUT triaging it exposed a harness-reading defect that changes the whole batch's story: eval_checkpoint.py's gait_valid = 'not sacrificed_legs' ONLY -- it does NOT get cleared by TERM, so 'gait_valid 12/12' has been silently misread as 'zero falls' all day. Re-checked the raw per-episode terminated/term_reason field on every completed seed via ops.sh report: seed7 (the M5-passing champion) actually shows 2 falls (walk/det/3 + walk/det/5, both tilt_roll, roll_peak 39-41deg) and seed23/s2 shows 2 falls (det/3 tilt_roll, sto/5 tilt_pitch) -- BOTH were previously verdicted/logged as clean 12/12 zero-falls. Corrected 7-seed tally: 7(2 falls)/23(2 falls)/13(1 fall+3 sac, already correctly caught)/31(1 fall)/37(1 fall)/41(3 falls) all UNSAFE by falls -- only seed43 (this run) is genuinely fall-free, 6/7. Sharper finding: 5 of the 6 fall-bearing seeds (7,23,13,31,37) fall at the SAME held-out episode index walk/det/3 with the same tilt_roll signature; seed43 itself nearly falls there too (roll_peak 17.6deg, classed 'recovered', the highest non-terminal roll_peak in its whole panel). Video (walk_det_3, several seeds) shows a stiffly-held/dragging leg (consistent with the permanently-baked per-episode weak/frozen-joint fault) during what reads as a turn-in-place segment, then a clean topple. This is NOT primarily a seed-basin lottery -- it is ONE specific held-out fault-type + turn-in-place command combination that the entire tipfrac05 recipe sits at or past its stability margin against, previously invisible because gait_valid was misread as a safety proxy. Do not promote tipfrac05 (or continue the acq1/kernelema budget lineage) as an M5 candidate on the strength of any 'gait_valid 12/12' claim without re-checking raw terminated fields. Full correction + evidence filed in CURRENT_TRUTHS.md and amp/STATUS.md this cycle. Evidence: logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_tipfrac05_{,seed{31,37,41,43},s2}_gate/report.json (raw terminated fields), walk_det_3_sheet.png (champion + siblings).

