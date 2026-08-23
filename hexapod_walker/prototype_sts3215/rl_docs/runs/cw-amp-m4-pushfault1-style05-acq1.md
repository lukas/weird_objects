# cw-amp-m4-pushfault1-style05-acq1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE-ceiling

**created**: 2026-08-23T02:51:41+00:00

**pod**: hexapod-mjx-train-0

**steps**: 6000000

**parent**: cw-amp-m4-pushfault1-style05

**wandb_id**: 9dwmq070

**hypothesis**: Plain English: style05's own push+fault-no-turn 2M discovery PASSED (gait_valid 12/12, zero sacrificed legs, 3/12 topples -- WASH-to-mild-help vs the noamp twin) with reward still rising every quarter (34->213) -- fund the same 6M acquisition dose the noamp twin (pushfault1-noamp-acq1, running/finished this cycle) and the solo-push style precedent (pushacq1-style05, PASSED acquisition: 1/6 det+0/6 sto topples) both used, to see whether style keeps its zero-sacrifice edge at full budget. Single lever: same checkpoint, +6M steps, dose unchanged.

**gate**: Acquisition (6M continuation, 8M total, DR-0, own cfg: fault+push both active, style kept). PASS = det+sto topples <=1/12 total, gait_valid >=11/12, zero sacrificed legs, det prog med >=0.9, style_reward_mean stays >0.1 (not collapsed), video shows compensation/limping not statues. Compare directly against the noamp twin's own acq1 result (same budget, same dose) to complete the style-vs-control read at acquisition scale, not just discovery. INFORMATIVE-ceiling = topples stay ~2-3/12 with reward flattening. FAIL = topples increase or gait_valid regresses below 9/12 with reward still rising.

**verdict**: Result: acquisition budget (8M total) does NOT close push+fault+style beyond its 2M discovery ceiling, and style now LOSES the style-vs-noamp comparison it was launched to complete. Evidence: own-cfg DR-0 gate (fault+push both on) lands 3/12 topples (2 det tilt_roll/tilt_pitch + 1 sto tilt_roll) -- the SAME total topple count as the 2M discovery (1 det + 2 sto), just reshuffled -- gait_valid held 12/12, zero sacrificed legs, style_reward_mean held 0.117 (>0.1 bar, not collapsed). But det prog med fell 1.21->0.65 and slip rose 2.77->4.84 (sto 0.87->0.71 prog, 3.59->4.84 slip) versus its own 2M discovery -- videos (det_0/1/2, sto_1/5) confirm the topples are genuine end-frame flips after 6+ clean walking strides (not statues) and the surviving episodes show honest six-leg gait, so this is not a new pathology, just no improvement. Reward flattened after Q1 (111->219->213->212), matching the gate's own pre-registered INFORMATIVE-ceiling branch (topples stay ~2-3/12, reward flattening) rather than PASS (needs <=1/12 topples, det prog>=0.9) or FAIL (needs topples increasing or gait_valid<9/12 with reward still rising -- neither happened). Completing the named style-vs-control comparison: the noamp twin's own acq1 (matched 8M budget, same dose) PASSED clean at 0/12 topples, det prog med 1.03, sto 0.81 -- style05 REVERSES its mild discovery-time edge (3/12 vs noamp's 2/12 at 2M) into a clear loss at acquisition scale (3/12 topples + worse prog/slip vs noamp's 0/12 + better prog/slip). Why: style-weighted reward apparently trades some task-progress optimization for the AMP discriminator term without buying extra topple-safety once the mechanism-safety bar is already close to solved by the task reward alone -- consistent with the track's running finding that style has been neutral-to-negative on every axis so far (never load-bearing). What's next: style05 is DEPRIORITIZED as an M4 push+fault carrier -- pushfault1-noamp-acq1 (already PASSED) is the substrate to build M5 candidates on; no further style-branch acquisition spend on this axis pair unless a future M5 composition specifically needs the discriminator online.

