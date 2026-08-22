# cw-dep-bcgait4-phasedir2-staged-fwd

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-22T04:05:23+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-dep-bcgait4-phasedir1

**wandb_id**: 8zkde4y7

**hardware_ready**: no

**hypothesis**: Retry RL on the phase-clock walking clone with the reward aligned to what we actually grade, starting from the easiest ask: this rung trains FORWARD-ONLY at the one fixed 0.08 m/s speed, and the new pricing must keep PPO from trading command obedience for generic stable fast walking (phasedir1's failure: reward up, dir_err 35.6->67.3, speed 0.068->0.139, slip/m 1.81->4.17). Aligned stack, preflight-proven (test_phasedir_semantics.py, 20 green): stride-averaged course charge (k_walk_course, EMA over one gait period so the ~35deg tick sway is NOT taxed), EMA-speed overspeed band (k_walk_course_overspeed), clone-banded loaded-slip gate+excess (loadslip_ok=2.2/max=4.0 around clone 1.56-2.07 vs degraded 4.17), anti-refusal travel floor (k_walk_idle_charge), plus STRONG gait preservation: phase-locked BC anchor to the clone's own teacher (bc_anchor_coef=1.0, bc_anchor_phase_lock=1 keeps the anchor gait on the policy-visible command-gated clock; bc_anchor_knee_abs=1 keeps it in the clone lineage's raw knee dialect). Staged curriculum per operator order fb_20260822T032514: forward-only first; heading set [0,+-45deg], full fixed headings, and irregular heading changes are LATER rungs, each launched only after the previous rung's clone-relative PASS. Prediction-if-true: RL preserves or improves the clone per the gate (no overspeed drift, slip flat, zero falls). Prediction-if-false: reward-behavior divergence again = the misalignment is not in these terms, STOP + operator.

**gate**: At 2M, DR-0, forward panel (eval_checkpoint det+sto with --cfg-set goal.walk_speed_min_m_s=0.08 goal.walk_speed_max_m_s=0.08 goal.walk_heading_max_rad=0.0) run on BOTH the final checkpoint AND the matched clone control ppo_goal_cw_bcgait_init_fullprof_phase1.zip (same seeds, same harness). CLONE-RELATIVE PASS ONLY (dir_err has a ~35deg tick-sway floor - judge deltas, never raw<=30 alone): (a) zero falls, gait_valid 6/6; (b) along-command progress >= 0.9x clone; (c) slip/m <= 1.15x clone; (d) dir_err med <= clone + 5deg; (e) speed_mean in [0.06,0.096] - no overspeed drift; (f) report roll_tail_deg/drag_m vs clone. VERDICT MUST tabulate reward components vs behavior (reward_walk_course, reward_walk_course_overspeed, reward_loadslip_excess, walk_loadslip_ratio, walk_course_cos): reward-up with ANY behavior axis worse than clone = FAIL (order item 6). PASS -> pre-registered rung B respec: goal.walk_heading_set=[0,0.7854,-0.7854], same gate per heading bin. FAIL falls/gait-break = charges destabilize despite anchor, STOP + operator. FAIL obedient-but-slow (<0.9x clone) = charges overpriced, STOP + report doses. NO DOWNLOAD_ANSWER change from this run.

**verdict**: FAIL (pre-registered obedient-but-slow branch, rung A forward-only): clone-relative det panel — falls 0/6=clone, gait_valid 6/6=clone, slip/m 2.00 vs clone 1.89 (1.06x, <=1.15x OK), dir_err 31.2 vs 35.5 (-4.4deg, OK), speed 0.060 in [0.06,0.096] at the floor, BUT along-progress 0.694m vs clone 0.830m = 0.836x < 0.9x. Charges overpriced: per-tick overspeed/loadslip charges taxed stochastic rollout noise (train std stuck 0.3675->0.3609; training loadslip_ratio ~5.1 > max 4.0 and overspeed -2.2/tick FLAT all run; sto panel shows clone pays the same noise slip 24-31/m) so the cheapest gradient was shrinking the mean gait. ep_rew fall -209->-1103 is an ep_len artifact (75->375); components flat; NOT reward-up-behavior-down. Video clean: level, all 6 legs cycling, no pathology. Rung B NOT launched per gate. STOP + doses reported.

