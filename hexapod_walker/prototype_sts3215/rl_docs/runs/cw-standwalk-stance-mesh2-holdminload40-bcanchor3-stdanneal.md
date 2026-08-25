# cw-standwalk-stance-mesh2-holdminload40-bcanchor3-stdanneal

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T12:41:25+00:00

**pod**: hexapod-mjx-train-1

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-holdminload40-bcanchor3

**wandb_id**: bmoh247p

**hypothesis**: Rung-7 FIRST-EVER det-mode PASS just landed (bcanchor3/bcanchor1: 6/6 valid_plant, height_err<=0.7mm, cur_max~1.1-1.3A, clean settled video) -- but BOTH doses still fail sto-mode 0/6, current pinning at 2.64A under the policy's own action noise (log-std-init=0, std~1.0, never annealed in this recipe). Same fix that unlocked the joystick track's stotight45 DONE-gate pass (annealing the trained noise floor down) should let the ALREADY-LEARNED honest six-foot plant survive the noise it currently cannot: does annealing log_std down (-4.0 final, matching stotight45's precedent) on top of the working bc_anchor=3.0 mechanism close the det/sto gap instead of the mechanism being reward-noise-limited?

**gate**: 8M continuation. Hold DR-0 det+sto n=6+6 AND own-DR(0.2) det+sto n=6+6. PASS: sto valid_plant >=4/6 at DR-0 (matching det's existing 6/6) with cur_p95<=1.5A, det stays >=5/6 (anneal must not regress the already-working det basin). PARTIAL: sto improves materially (1-3/6, or cur_max trending down from 2.64A) but short of the bar -- dose/budget follow-up. FAIL: sto stays 0/6 pinned at 2.64A unchanged -- noise-floor is not the sto blocker, escalate to a DR-0.2-in-training exposure fix or accept det-only as this rung's ceiling and move to composing rise/lower on top of the det-clean hold checkpoint.

