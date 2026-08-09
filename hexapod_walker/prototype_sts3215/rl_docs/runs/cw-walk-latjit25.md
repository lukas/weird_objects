# cw-walk-latjit25

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T16:26:45+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: k26wfzbl

**hypothesis**: OPERATOR WISHLIST 13b (richer physics variation, one axis per run) via the cycle-49 dr.<field> overrides: command latency jitter is the cheap-servo sim2real axis most likely to bite on hardware (STS3215 bus timing varies). ISOLATED axis: dr-scale 0.0 with ONLY dr.latency_scale=0.5,2.5 randomized (0.5x-2.5x fitted servo latency, wider than the standard 0.7-1.8 envelope) - one variable off the no-DR champion. If-true: gait holds across the latency spread (own-cfg harness gv 12/12, 0 term, det median fwd >=1.2m @30s) and DR0 retention holds - latency robustness is trainable by exposure and joins the transfer recipe. If-false: high-latency draws destabilize the gait (terminations/prog craters) - latency needs history/estimator machinery (contact-aux or DreamWaQ rung), not exposure. Strongest alternative: policy slows cadence to survive latency, hiding as pass - check stride/cadence vs champion.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.latency_scale=0.5,2.5, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2 m; plus DR0 no-jitter retention det 6/6 gv, det slip/m within champion band (<=1.24); frames watched det

