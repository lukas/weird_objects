# cw-amp-m4-pushfault1-noamp-acq1-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-23T02:36:53+00:00

**pod**: hexapod-mjx-train-0

**steps**: 6000000

**parent**: cw-amp-m4-pushfault1-noamp-r2

**hypothesis**: Plain English: push+fault-without-turn cleared its mechanism-safety bar at 2M discovery (gait_valid 11/12, only 2/12 topples -- fewer than solo-push's own 4/12 at matched budget) with reward still rising every quarter (23->281) -- per the 08-21 ruling this is undertrained, not misaligned, so fund the same 6M acquisition dose pushacq1-noamp used to close its own single-axis gap outright. Single lever: same checkpoint, +6M steps, dose unchanged.

**gate**: Acquisition (6M continuation, 8M total, DR-0, own cfg: fault+push both active). PASS = det+sto topples <=1/12 total (matching pushacq1-noamp's own acquisition close: 0/12 after 6M), gait_valid >=11/12 held or improved, zero sacrificed legs, det prog med >=0.9, video shows compensation/limping not statues on any faulted leg. INFORMATIVE-ceiling = topples stay ~2/12 with reward flattening -- 2M discovery numbers were already near the ceiling, budget does not buy further margin (informs whether M4's remaining gap is turn-specific only). FAIL = topples increase or gait_valid regresses below 9/12 with reward still rising -- would be a genuine reward/eval misalignment to dig into.

**refused_reason**: acquisition runs require --evidence: name the healthy canary and a comparable full-budget learning precedent.

