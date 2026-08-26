# cw-standwalk-stance-mesh2-standheight-rung5-acq8m-segfix-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: ACQUISITION PASS (own-scope) - JOINT DIVERGENCE

**created**: 2026-08-26T08:04:56+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-standheight-rung5-acq8m-s1

**wandb_id**: i50v00p9

**hypothesis**: Seed-1 twin of the segment-window widening test (9-11s vs default 6-8s): does it fix seed1's own sharper flat-start rise residual (7/12 in the corrected-contract seqprobe, mostly hold_low_height stalls on flat draws)? Same code-read root cause as the seed-0 twin: rise's own schedule needs >=7.0s and the default segment draw sometimes lands shorter. Continuation off seed-1's own acq8m checkpoint.

**gate**: PASS: composed seqprobe's flat-start rise sub-count improves (fewer hold_low_height terms) with hold/lower staying >= this seed's own acq8m level (6/6+6/6 lower, 6/6+6/6 hold). FAIL: unchanged/worse -- joint with the seed-0 twin per the same disagreement convention this campaign uses throughout.

**verdict**: Own scope (seed 1): the segment-window widen (mode_seq_segment_s_min/max 6-8s->9-11s) CONFIRMS its hypothesis decisively. Composed seqprobe (mixed kinds, registered instrument): hold 6/6+6/6 zero-term, lower 6/6+6/6 zero-term herr 0.1-0.8mm (tighter than acq8m parent's 14.1-14.9mm), rise det4/6+sto6/6=10/12 (2 OC, no flat draws in this n=12 mixed sample -- rsi/crouch/bridge only, confirmed per-episode). Built+ran a DEDICATED flat-pinned composed probe this cycle (goal.rise_flat_frac=1.0/partial=0/rsi=0 on top of the same mode_seq+segment override, n=12 det+sto) on this ckpt AND its acq8m-s1 parent for a direct before/after: acq8m-s1 (pre-fix) = 2/12 valid, 10/12 hold_low_height terms, herr 48-72mm on fails -- the severe flat-in-composition failure this arm was funded to fix. segfix-s1 (this run) = 12/12 valid, ZERO terms, herr 0.3-3.6mm, video-confirmed genuine splay->tuck->full-stand every draw. 2/12->12/12, clean win on this seed's own question. JOINT CALL: DIVERGENCE, not a clean pass -- ran the identical matched flat-pinned probe against the seed-0 twin (-segfix, a concurrent cycle's run; read-only diagnostic evidence-gathering only, not a verdict on their behalf) and ITS acq8m parent: acq8m (seed0) flat-pinned = 12/12 valid, ZERO terms, herr 0.3-1.9mm (seed0 had NO flat-in-composition problem pre-fix); segfix (seed0) flat-pinned = 9/12 valid, 3 NEW hold_low_height terms (herr 49.2-52.5mm on fails), video-confirmed genuine stall-low (crouched terminal frame, never completes the last stretch to full height). The identical widen that fixes seed1 (2/12->12/12) REGRESSES seed0 (12/12->9/12) -- the gate's own PASS branch (seed1) and its FAIL branch ('unchanged or worse', seed0) BOTH fire on the same arm, split by seed. Root cause open: the Next-0.5 root-cause note's 'widening a time-starved first segment can only help' logic does not explain a NEW failure appearing where none existed pre-fix. Recommendation: do NOT promote the widened segment window as a blanket replacement for the acq8m recipe; acq8m/acq8m-s1 remain the standing stage-1 checkpoints pending root-cause. DIG-IN flagged: on-pod per-tick trace of segfix-seed0's failed flat episodes vs its acq8m-parent equivalents (does the flat-time-indexed BC-anchor clock, tuned to a ~7s reference, mis-pace inside a 9-11s segment for a subset of draws in a way the old 6-8s window didn't hit?). Evidence: logs/ckpt_eval/cw_standwalk_stance_mesh2_standheight_rung5_acq8m_segfix_s1_{gate,owncfg,seqprobe,seqprobe_flat}/ (this run, own scope); .../_segfix_{seqprobe,seqprobe_flat}/ (sibling, cited evidence only); .../_acq8m_seqprobe_flat/ + _acq8m_s1_seqprobe_flat/ (matched pre-fix baselines, built this cycle). W&B i50v00p9.

