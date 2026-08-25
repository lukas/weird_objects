# cw-standwalk-stance-mesh2-holdminload40-bcanchor3-stdanneal

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED-PASS

**created**: 2026-08-25T12:41:25+00:00

**pod**: hexapod-mjx-train-1

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-holdminload40-bcanchor3

**wandb_id**: bmoh247p

**hypothesis**: Rung-7 FIRST-EVER det-mode PASS just landed (bcanchor3/bcanchor1: 6/6 valid_plant, height_err<=0.7mm, cur_max~1.1-1.3A, clean settled video) -- but BOTH doses still fail sto-mode 0/6, current pinning at 2.64A under the policy's own action noise (log-std-init=0, std~1.0, never annealed in this recipe). Same fix that unlocked the joystick track's stotight45 DONE-gate pass (annealing the trained noise floor down) should let the ALREADY-LEARNED honest six-foot plant survive the noise it currently cannot: does annealing log_std down (-4.0 final, matching stotight45's precedent) on top of the working bc_anchor=3.0 mechanism close the det/sto gap instead of the mechanism being reward-noise-limited?

**gate**: 8M continuation. Hold DR-0 det+sto n=6+6 AND own-DR(0.2) det+sto n=6+6. PASS: sto valid_plant >=4/6 at DR-0 (matching det's existing 6/6) with cur_p95<=1.5A, det stays >=5/6 (anneal must not regress the already-working det basin). PARTIAL: sto improves materially (1-3/6, or cur_max trending down from 2.64A) but short of the bar -- dose/budget follow-up. FAIL: sto stays 0/6 pinned at 2.64A unchanged -- noise-floor is not the sto blocker, escalate to a DR-0.2-in-training exposure fix or accept det-only as this rung's ceiling and move to composing rise/lower on top of the det-clean hold checkpoint.

**verdict**: The robot can now stand rock-solid on all six feet on the new heavier mesh model, even with action noise and physics randomization -- the first fully robust stance hold of the mesh era. 8M acquisition off the bcanchor3 canary (BC pose-anchor dose 3.0) with log-std annealed 0->-4.0 (final std 0.018): DR-0 det 6/6 AND sto 6/6 valid_plant (gate bar was sto>=4/6, det>=5/6), own-DR(0.2) det 6/6 AND sto 6/6, ZERO terminations in all 24 episodes, cur_p95 0.44-0.69A (trip line 2.5A), height_err_end 0.1-0.4mm, plant_margin ~150mm. Det and sto videos: level chassis, six loaded feet, motionless to truncation -- no crouch, no creep, no flag leg. Reward rose every quarter (-14.6/100.7/290.9/553.9). This confirms std-annealing alone closed the det/sto gap the 2M canaries showed (their sto 0/6 was pure un-annealed noise-floor, as hypothesized). WHY: the supervised BC anchor gives the gradient a target posture six rungs of pure reward/termination shaping could not express; annealing removes the exploration noise that was shoving feet off their load. NEXT: hold rung CLOSED -- checkpoint ppo_goal_cw_standwalk_stance_mesh2_holdminload40_bcanchor3_stdanneal is the mesh hold champion/teacher; stage-1 moves to rise/lower (BC-anchor chain machinery exists but its 3 rise/lower bank tests are pinned to primitive heights and must be mesh-recalibrated before any mechanism launch). Hardware-ready: sim-only track, but behavior quality yes.

