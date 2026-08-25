# cw-standwalk-stance-mesh2-holdterm40

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T09:40:49+00:00

**pod**: hexapod-mjx-train-0

**steps**: 6000000

**parent**: cw-standwalk-stance-mesh2-holdload1min

**wandb_id**: ucrtk5um

**hypothesis**: Plain English: does ending the episode the moment the body collapses toward its belly (and pricing that like a real fall) finally keep the from-scratch policy standing on the six feet it starts on? Single lever off the holdload1min scratch recipe (min-load income, pricing untouched): safety.hold_max_height_drop_mm=40 + 1s grace terminates hold episodes whose chassis drops >40mm below the settled plant (reason hold_low_height, priced by the existing safety penalty + term_cost). Rationale: 5/5 load-min arms + 3/3 read product arms all walk OUT of the paying plant within ~0.7M steps, and the winning basins are QUIET SURVIVORS (belly-flop freeze / cold belly-rest: zero terminations, 0.5A) that absorb the policy -- op ruling 08-24: absorbing states beat prices, must come WITH a termination. Terminating basin exit resets envs back INTO the paying basin and makes time-in-basin a MONOTONE return axis (HOLD_BASIN_TERM bank, 5/5 green on primitive AND mesh: flop terminates <4s w/ right reason, quiet byte-identical, quiet > flop@11s > flop@6s > flop@1.5s). Prediction-if-true: hold panel >=10/12 valid six-foot plant by 6M, terminations high early then falling. Prediction-if-false: policy escapes into a DIFFERENT terminating basin (rear-up OC / tilt) or terminations stay pinned with reward flat -- then height-drop is the wrong trigger and the pre-registered follow-up is a min-load-based exit trigger (terminate when min foot load stays floor-pinned), bank first. Strongest alternative cheat: hovering just above the 40mm line (h_err ~30-39mm crouch) -- watch eval height_err_end and hold_load_factor.

**gate**: Hold panel at 6M: pod_eval hold DR-0 det+sto n=6+6. PASS: >=10/12 survive 15s with zero over_current/tilt/hold_low_height terminations AND det episodes show six-foot stance (valid_plant true, or all-leg duty>=0.9 with end_clear<5mm) AND cur_p95<=1.0A; own-DR(0.2) read alongside. FAIL: still exits the basin (any escape family) or crouch-hovers above the termination line without valid plant.

