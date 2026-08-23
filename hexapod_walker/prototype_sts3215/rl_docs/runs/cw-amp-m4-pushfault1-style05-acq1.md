# cw-amp-m4-pushfault1-style05-acq1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T02:51:41+00:00

**pod**: hexapod-mjx-train-0

**steps**: 6000000

**parent**: cw-amp-m4-pushfault1-style05

**wandb_id**: 9dwmq070

**hypothesis**: Plain English: style05's own push+fault-no-turn 2M discovery PASSED (gait_valid 12/12, zero sacrificed legs, 3/12 topples -- WASH-to-mild-help vs the noamp twin) with reward still rising every quarter (34->213) -- fund the same 6M acquisition dose the noamp twin (pushfault1-noamp-acq1, running/finished this cycle) and the solo-push style precedent (pushacq1-style05, PASSED acquisition: 1/6 det+0/6 sto topples) both used, to see whether style keeps its zero-sacrifice edge at full budget. Single lever: same checkpoint, +6M steps, dose unchanged.

**gate**: Acquisition (6M continuation, 8M total, DR-0, own cfg: fault+push both active, style kept). PASS = det+sto topples <=1/12 total, gait_valid >=11/12, zero sacrificed legs, det prog med >=0.9, style_reward_mean stays >0.1 (not collapsed), video shows compensation/limping not statues. Compare directly against the noamp twin's own acq1 result (same budget, same dose) to complete the style-vs-control read at acquisition scale, not just discovery. INFORMATIVE-ceiling = topples stay ~2-3/12 with reward flattening. FAIL = topples increase or gait_valid regresses below 9/12 with reward still rising.

