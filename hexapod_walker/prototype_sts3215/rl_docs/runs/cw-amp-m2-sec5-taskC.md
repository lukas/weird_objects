# cw-amp-m2-sec5-taskC

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T19:12:18+00:00

**pod**: hexapod-mjx-train-8

**steps**: 2000000

**parent**: cw-amp-m2-styleonly-v2-c1b

**hypothesis**: Plain English: does the AMP brief's own literal section-5 minimal task reward (plain Gaussian velocity-tracking kernel + linear progress + the shared upright/height/action/current regularizers, zero of the SLIPWALK anti-slip/anchor/idle-charge/gait-gate/drag-stance apparatus every one of the 9 prior from-scratch M2 arms reused, plus the already-validated reward.term_penalty=400 anti-suicide fix and goal.walk_pure=1.0 to close the M2-c1 mixed-goal-mode statue exploit) let a from-scratch policy actually walk, and does AMP style help on top of it? Executes the pre-registered brief-5.2 A/B/C task/style sweep (0.7/0.3, 0.5/0.5, 0.3/0.7) on AMP_MINIMAL_OVERRIDES (rl_move/tests/test_task_semantics.py, 4/4 new tests + 178/179 full bank green): the scripted-twin bank already confirms real travel beats every stationary/sliding twin by a modest ~230-250/ep margin and dying fast is priced far below surviving, but that trying (stall) and refusing (park) are barely separated under the task reward alone -- AMP style is the candidate source of that missing 'do something coordinated' gradient (already shown this cycle to organize real leg-cycling on its own, cw-amp-m2-styleonly-v2-c1b, though with zero net-displacement incentive). Single held envelope (speed 0-0.25 m/s, yaw +/-0.5 rad/s, matching the retired freeprog family for comparability) across the 3-arm sweep; task/style ratio is the only variable. (this arm: task 0.3 / style 0.7, brief mixture C)

**gate**: Discovery (2M steps, judged on det video (3+ episodes per arm) + gait_valid/fwd-travel harness numbers + amp/style_reward_mean, d_real/d_fake, NOT the joystick DONE gate). INFORMATIVE-PASS (any arm) = det video shows real net forward travel (>=0.10 m/15s) with visibly cyclic multi-leg contact/swing, discriminator unsaturated. FAIL-same-statue = frozen/half-tripod/march-in-place basin persists (gait_valid 0/6 or march-in-place with ~0 net travel) despite dropping the whole SLIPWALK stack -- closes the reward-architecture-alone hypothesis and points to task restructuring or a BC-pretrain phase as the next lever, per q_20260822T1815Z. Compare all 3 arms + the AMP_MINIMAL bank's own scripted-twin numbers; select on video first, reward second.

