# cw-amp-m3-pushhard1-noamp-n2040-c1r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T01:28:13+00:00

**pod**: hexapod-mjx-train-2

**steps**: 6000000

**parent**: cw-amp-m3-pushhard1-noamp-n2040

**wandb_id**: yqxt59me

**hypothesis**: Plain English: does the 20-40N hard-shove walker just need MORE BUDGET, or was the flat dose jump the mistake? Retry of pushhard1-noamp-n2040-c1, which never trained (REFUSED 3x on a stale pod code marker — infra, not spec; this is the single permitted retry). Continues pushhard1-noamp-n2040 for 6M more at the SAME 20-40N dose from its own checkpoint (08-21 ruling: reward was still rising 84->115 in the final quarter and tilt-roll terms still falling 46->43 at cutoff), as the matched budget control for the force-curriculum chain (pushcur2-noamp-n2040, launched same cycle, 12M total each). Prediction-if-true (undertrained): tilt terms resume falling and the 20-40N gate reaches topples <=1/6 det + <=2/6 sto — flat-jump-plus-budget suffices, curriculum unnecessary. Prediction-if-false: reward inflates on surviving episodes while terms and topples stay at the ~4/12 plateau — the rare-hard-shove gradient is the bottleneck and the curriculum wins the fork. Strongest alternative: both this and the staged chain plateau at the same topple floor, pointing to a recovery-mechanism gap rather than a training-signal gap.

**gate**: Continuation control (6M more at 20-40N from n2040's own ckpt; judged vs pushcur2-noamp-n2040 at matched 12M total). PASS = own-cfg gate topples <=1/6 det AND <=2/6 sto, gait_valid >=5/6 det+sto, det prog med >=0.9, zero sacrificed, genuine recovery on video. INFORMATIVE-plateau = topples stay ~4/12 with tilt terms flat over last 2M => budget refuted, curriculum-vs-recovery fork decided by the staged chain. INFORMATIVE-ceiling = terms still falling at cutoff => continue per 08-21. FAIL = collapse/statue/NaN.

