# cw-amp-m3-pushhard1-noamp-n2040-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-23T00:42:08+00:00

**pod**: hexapod-mjx-train-1

**steps**: 6000000

**parent**: cw-amp-m3-pushhard1-noamp-n2040

**hypothesis**: Plain English: does the 20-40N hard-shove walker just need MORE BUDGET, or was the flat dose jump the mistake? This continues pushhard1-noamp-n2040 for 6M more at the SAME 20-40N dose from its own checkpoint (08-21 ruling: reward was still rising 84->115 in the final quarter and tilt-roll terms still falling 46->43 at cutoff), as the matched control for the force-curriculum bridge (pushcur1-noamp-b1530, launched same cycle). Prediction-if-true (undertrained): tilt terms resume falling and the gate at 20-40N reaches topples <=1/6 det + <=2/6 sto — flat-jump-plus-budget suffices, curriculum unnecessary. Prediction-if-false: reward keeps inflating on surviving episodes while tilt terms and gate topples stay at the plateau (~4/12) — the rare-hard-shove gradient is the bottleneck, curriculum wins the fork. Strongest alternative: both this and the bridge plateau at the same topple floor, pointing to a recovery-mechanism gap (can't recover once past tilt threshold) rather than a training-signal gap.

**gate**: Continuation (+6M at 20-40N, DR-0, single shove). Judged head-to-head vs cw-amp-m3-pushcur1-noamp-b1530's stage-2 chain at matched total budget. PASS = DR-0 own-cfg gate gait_valid >=5/6 det+sto, zero sacrificed legs, det prog med >=0.9, topples <=1/6 det AND <=2/6 sto. INFORMATIVE-plateau = topples still ~4/12 with terms flat over last 2M => flat-jump-plus-budget refuted, curriculum line owns the axis. FAIL = collapse/statue/NaN.

**refused_reason**: hexapod-mjx-train-1 code marker c96540f45785701f8fbb71aedc9b175437a8d1c0 != local HEAD 07d79522e11398fbb316420fb0efdc6d7d589005. Sync first: snapshot.sh --sync hexapod-mjx-train-1 (and snapshot/commit before that if the tree is dirty).

