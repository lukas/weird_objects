# cw-dep-bcgait4-phasedir3-fwd-reprice

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-22T05:38:30+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-dep-bcgait4-phasedir2-staged-fwd

**hypothesis**: Relaunch of the fwd-only rung A with the reward repriced so PPO's own exploration noise is no longer taxed: phasedir2-staged-fwd kept the gait perfectly (anchor win) but shrank it (progress 0.836x clone) because the det-band charges billed the stochastic rollout itself — training loadslip_ratio ~5.1 > max 4.0 zeroed the income gate ALL RUN (reward_walk 0.84->0.06/tick) plus a flat -1.19/tick excess charge, and the |v_ema| overspeed band fired ~-3/tick on noise sway (walk_speed 0.12 while along-EMA was ~0.008). Repriced stack, measurement-calibrated and preflight-proven (test_phasedir_semantics.py 24/24, incl. 4 new NOISY-REGIME tests at the measured stuck std 0.36): loadslip ok/max 7.0/10.0 (above the measured noisy-clone band 4.8-6.4; noisy clone keeps factor ~1 and pays ~0, gross skating still gated+charged), overspeed on the unbiased ALONG-command projection of the course EMA (walk_course_overspeed_along=1, new default-off cfg; noise sway pays 0, sustained directed overspeed pays in full), course charge floored at 0.04 m/s smoothed speed (noise-direction ticks unpriced; wrong-way travel at command speed still pays hard). Preflight sign-flip proven: noisy obey ~112 > noisy shrunken-gait(0.75x) ~95 (was inverted), > noisy stall ~45 and park ~102. Same strong phase-locked BC anchor, same fwd-only 0.08 m/s rung A, FRESH from the phase clone (no continuation, per operator focus fb 20260822T051709Z). KNOWN HOLE recorded (q_20260822T0640Z): at std 0.36 mean-overdrive out-earns the noisy clone by ~10 (overdrive compensates noise; unpriceable behaviorally since the attractor's det slip 2.39 < noisy clone 5.91 on the same scalar) — containment is the anchor + gate items (c)/(e). Prediction-if-true: income restored -> RL preserves clone progress (>=0.9x) with zero falls and no overspeed drift. Prediction-if-false: det speed drifts >0.096 = the known hole won over the anchor -> next lever is anchor dose or std annealing, not new charges.

**gate**: At 2M, DR-0, forward panel (eval_checkpoint det+sto with --cfg-set goal.walk_speed_min_m_s=0.08 goal.walk_speed_max_m_s=0.08 goal.walk_heading_max_rad=0.0) on BOTH the final checkpoint AND the matched clone control ppo_goal_cw_bcgait_init_fullprof_phase1.zip (same seeds/harness). CLONE-RELATIVE PASS ONLY: (a) zero falls, gait_valid 6/6; (b) along-command progress >= 0.9x clone; (c) slip/m <= 1.15x clone; (d) dir_err med <= clone + 5deg; (e) speed_mean in [0.06,0.096]; (f) report roll_tail_deg/drag_m vs clone. VERDICT MUST tabulate reward components vs behavior (reward_walk_course, reward_walk_course_overspeed, reward_loadslip_excess, walk_loadslip_ratio, walk_loadslip_factor, walk_course_cos) — training walk_loadslip_factor must sit ~1 this time; factor <0.5 = repricing failed, say so. PASS -> pre-registered rung B respec: goal.walk_heading_set=[0,0.7854,-0.7854], same gate per heading bin. FAIL det-overspeed (speed>0.096 or slip>1.15x) = the recorded known hole (q_20260822T0640Z) beat the anchor -> lever is anchor dose/std annealing, NOT new behavior charges. FAIL obedient-but-slow again (<0.9x clone) with factor ~1 and charges ~0 = the shrink was never charge-driven -> STOP, rethink income side. NO DOWNLOAD_ANSWER change from this run.

**refused_reason**: acquisition runs require --evidence: name the healthy canary and a comparable full-budget learning precedent.

