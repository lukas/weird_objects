# cw-amp-m2-sec5-taskA

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-22T19:03:29+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-styleonly-v2-c1b

**hypothesis**: Plain English: does the AMP brief's own literal section-5 minimal task reward (plain Gaussian velocity-tracking kernel + linear progress + the shared upright/height/action/current regularizers, zero of the SLIPWALK anti-slip/anchor/idle-charge/gait-gate/drag-stance apparatus every one of the 9 prior from-scratch M2 arms reused, plus the already-validated reward.term_penalty=400 anti-suicide fix and goal.walk_pure=1.0 to close the M2-c1 mixed-goal-mode statue exploit) let a from-scratch policy actually walk, and does AMP style help on top of it? Executes the pre-registered brief-5.2 A/B/C task/style sweep (0.7/0.3, 0.5/0.5, 0.3/0.7) on AMP_MINIMAL_OVERRIDES (rl_move/tests/test_task_semantics.py, 4/4 new tests + 178/179 full bank green): the scripted-twin bank already confirms real travel beats every stationary/sliding twin by a modest ~230-250/ep margin and dying fast is priced far below surviving, but that trying (stall) and refusing (park) are barely separated under the task reward alone -- AMP style is the candidate source of that missing 'do something coordinated' gradient (already shown this cycle to organize real leg-cycling on its own, cw-amp-m2-styleonly-v2-c1b, though with zero net-displacement incentive). Single held envelope (speed 0-0.25 m/s, yaw +/-0.5 rad/s, matching the retired freeprog family for comparability) across the 3-arm sweep; task/style ratio is the only variable. (this arm: task 0.7 / style 0.3, brief mixture A)

**gate**: Discovery (2M steps, judged on det video (3+ episodes per arm) + gait_valid/fwd-travel harness numbers + amp/style_reward_mean, d_real/d_fake, NOT the joystick DONE gate). INFORMATIVE-PASS (any arm) = det video shows real net forward travel (>=0.10 m/15s) with visibly cyclic multi-leg contact/swing, discriminator unsaturated. FAIL-same-statue = frozen/half-tripod/march-in-place basin persists (gait_valid 0/6 or march-in-place with ~0 net travel) despite dropping the whole SLIPWALK stack -- closes the reward-architecture-alone hypothesis and points to task restructuring or a BC-pretrain phase as the next lever, per q_20260822T1815Z. Compare all 3 arms + the AMP_MINIMAL bank's own scripted-twin numbers; select on video first, reward second.

**verdict**: FAIL-same-statue, arm A (task/style 0.7/0.3) of the pre-registered sec5 A/B/C grid on the section-5 minimal reward. Evidence: DR-0 gate det walk 0/6, gait_valid 0/6 (legs [3,5] sacrificed), prog_ratio 0.02, slip/m 9.41, dir_err 68.7deg, speed 0.008 m/s; sto gait_valid 4/6 but slip/m 15.20, dir_err 77.1deg, prog_ratio 0.03, only 3/6 settled. Video (6 det + 6 sto episodes) shows a crouched, near-static pose shuffling legs with no net translation -- march-in-place/skating, not walking, far under the 0.10m/15s informative-pass bar. Why: training reward DECLINED every quarter (-39.3/-44.6/-72.5/-74.2) -- not the 08-21 rising-reward-misaligned case, a genuine not-learning result at this dose. Mechanism trace: env/height_err_mm jumped 5.8->87.7mm in the first quarter (policy actively crouches away from the upright target early) and only partially recovered to 58.5mm by 2M; amp/style_reward_mean rose slowly (0.009->0.106) with the discriminator healthy and unsaturated (d_real 0.78/d_fake -0.95) but too small a share of the blend (task_w 0.7) to rescue the crouch. What's next: read jointly with taskB (0.5/0.5, same march-in-place pattern on early peek: det gait_valid 3/6 legs[0,4] sacrificed, sto gait_valid 6/6 but slip 13.88/dir 76.1 -- also FAIL-same-statue) and taskC/noamp (both finishing now, owned by a concurrent cycle) before closing the reward-architecture-alone hypothesis per q_20260822T1815Z; do not launch further reward-dose arms on this grid until that joint read lands.

