# cw-standwalk-stance-mesh2-standheight-rung5-acq8m-segfix

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL - MECHANISM REFUTED

**created**: 2026-08-26T08:00:42+00:00

**pod**: hexapod-mjx-train-6

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-standheight-rung5-acq8m

**wandb_id**: a9jnk4dr

**hypothesis**: Does widening the composed mode_seq segment window (9-11s vs the default 6-8s) fix the flat-start rise failures the corrected-contract seqprobe revealed? Code-read root cause: this recipe's own rise schedule needs >=7.0s (1s hold + rise_ramp_s=6.0) to reach the commanded height once, but the default segment draw U(6,8) lands under 7.0s roughly half the time when rise is the sequence's first segment, cutting a flat start's full physical climb short before it completes and riding a still-low height into the next segment (hold_low_height fires there). Same mechanism class as this file's already-diagnosed lower-phase segment-timing residual. Continuation off the promoted acq8m checkpoint (seed 0), no skill relearning needed, pure timing-budget test.

**gate**: PASS: composed seqprobe's flat-start rise sub-count improves (fewer hold_low_height terms on flat/bridge draws specifically) with hold/lower staying >= the acq8m checkpoint's own level (no new majority term). FAIL: flat sub-count unchanged or worse -- refutes the segment-timing hypothesis, points to genuine skill interference requiring a different (multi-teacher/KL) mechanism, dig-in scope.

**verdict**: Result: widening the composed mode_seq segment window (9-11s vs default 6-8s) does NOT fix flat-start rise -- it makes seed-0's flat-start rise WORSE, not better, refuting the hypothesis this arm was funded to test. Evidence (own-scope, seed 0; joint call pends the -s1 twin, held by a concurrent cycle): composed seqprobe (mode_seq_stance+hold_height_cmd, per-mode 6 det+sto) drew ZERO flat starts by chance (bridge/crouch/rsi only: rise det 4/6, sto 6/6), so I ran a targeted flat-forced probe (goal.rise_flat_frac=1.0/partial=0/rsi=0, same composed context, same 9-11s window this checkpoint trained under): flat-start rise det 4/6 + sto 5/6 = 9/12 (75%), all 3 fails the EXACT predicted hold_low_height stall (herr 49-52mm -- rise cut short, low height carried into the next segment). Matched-parent control: the unmodified acq8m checkpoint (seed 0, native 6-8s window), same flat-forced probe: 11/12 (my own run, 1 borderline hold_low_height at herr=1.7mm) and 12/12 (a concurrent cycle's independent re-run) -- seed-0's flat-start rise was ALREADY near-solved in the composed context before this lever, and the widened window regressed it. hold/lower stayed clean or improved (hold sto 6/6 vs acq8m's own 5/6; lower 6/6+6/6 zero-term both) -- no regression there, isolating the flat-rise regression as the lever's real effect. Why: the root-cause read (widened window should give the flat climb -- which needs >=7.0s -- more room before the next segment's hold_low_height check fires) was directionally right about the mechanism but wrong about the fix; wider segments evidently interact with something else in the composed schedule (mid-sequence habituation, or hold_low_height's own grace/threshold, or a training-time distribution shift from spending 2M steps at the new segment lengths) that costs more than the timing margin buys. What's next: this refutes the segment-timing lever per its own pre-registered FAIL branch -- do not spend more budget on segment-window dosing. The next lever is a genuine skill-interference fix per the file's own Next list (a height-cmd/hold_low_height-aware bc_anchor loosening, or a matched multi-teacher/KL mechanism), dig-in scope, and should be taken up once the -s1 joint call closes (its own flat-forced probe was still copying back as of this cycle).

