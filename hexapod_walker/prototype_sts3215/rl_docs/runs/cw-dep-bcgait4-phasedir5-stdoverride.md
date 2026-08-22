# cw-dep-bcgait4-phasedir5-stdoverride

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-22T06:30:26+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-dep-bcgait4-phasedir4-entanneal

**wandb_id**: io3m0yr7

**hypothesis**: Same fwd-only obedience task and repriced loadslip/overspeed reward as phasedir3/phasedir4 (unchanged), but instead of relying on the entropy coefficient to indirectly shrink PPO's stuck action std over training (phasedir4 FAIL: ent_coef annealed 10x but std only moved 0.368->0.352), directly RESET the warm-started policy's log_std to -2.0 (std=0.135, well below the clone's own noisy 0.368) before training starts, using the new default-off train_ppo_mjx --warm-log-std-override (built+on-pod-smoke-tested this cycle: verified it lands exactly at the requested value and is a no-op when unset). If the noise-band theory from phasedir3/4 is right, a policy that starts and stays much more deterministic should let det-mode slip/progress track the clone tightly without the wide noisy-clone-sized loadslip band being a problem, since realized noise is now small everywhere. Prediction-if-true: det slip <=1.15x clone AND progress >=0.9x clone, zero falls, gait 6/6 -> PASS, launch rung B (heading-set respec). Prediction-if-false-collapsed-exploration: reward flat/falling from step 0 with high early falls or a frozen/degenerate gait (std too low to explore away from a still-imperfect warm start) -> the override is too aggressive, retry with a milder value (e.g. -1.5). Prediction-if-false-still-misses-cleanly: zero falls, gait_valid 6/6, low realized std confirmed in eval report, but slip/progress STILL miss the clone bars -> the sto/det-noise-band theory is refuted outright (not just weakly tested), DIG-IN required (per-leg gait video) before any further reward-mechanism edit.

**gate**: At 2M, DR-0, forward panel (eval_checkpoint det+sto with --cfg-set goal.walk_speed_min_m_s=0.08 goal.walk_speed_max_m_s=0.08 goal.walk_heading_max_rad=0.0) on the final checkpoint, clone-relative against the SAME control used for phasedir3/4 (ppo_goal_cw_bcgait_init_fullprof_phase1.zip, logs/ckpt_eval/phasedir3_clone_control_gate -- do not re-run it, same seeds/harness). CLONE-RELATIVE PASS ONLY: (a) zero falls, gait_valid 6/6; (b) along-command progress >= 0.9x clone; (c) slip/m <= 1.15x clone; (d) dir_err med <= clone + 5deg; (e) speed_mean in [0.06,0.096]; (f) report roll_tail_deg/drag_m vs clone. VERDICT MUST report policy_std from the eval report and state explicitly whether it actually dropped (target: well below 0.352, ideally <0.2) before drawing any conclusion. PASS -> pre-registered rung B respec: goal.walk_heading_set=[0,0.7854,-0.7854], same gate per heading bin. FAIL with std still not shrunk in eval (override didn't take / got overwhelmed by training) = escalate override magnitude or investigate why. FAIL with std confirmed low (<0.2) and gate still missed = the noise-band theory is REFUTED, DIG-IN required (per-leg gait metrics + video) before any further reward edit -- do not launch another anneal/override variant blind. FAIL with collapsed/degenerate behavior (frozen gait, early falls, reward cratering from step 0) = override too aggressive, retry milder (-1.5) as phasedir6, not a theory refutation. NO DOWNLOAD_ANSWER change from this run.

