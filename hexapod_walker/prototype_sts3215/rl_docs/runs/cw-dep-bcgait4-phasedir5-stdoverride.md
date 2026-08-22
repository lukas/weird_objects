# cw-dep-bcgait4-phasedir5-stdoverride

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-22T06:30:26+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-dep-bcgait4-phasedir4-entanneal

**wandb_id**: io3m0yr7

**hardware_ready**: False

**hypothesis**: Same fwd-only obedience task and repriced loadslip/overspeed reward as phasedir3/phasedir4 (unchanged), but instead of relying on the entropy coefficient to indirectly shrink PPO's stuck action std over training (phasedir4 FAIL: ent_coef annealed 10x but std only moved 0.368->0.352), directly RESET the warm-started policy's log_std to -2.0 (std=0.135, well below the clone's own noisy 0.368) before training starts, using the new default-off train_ppo_mjx --warm-log-std-override (built+on-pod-smoke-tested this cycle: verified it lands exactly at the requested value and is a no-op when unset). If the noise-band theory from phasedir3/4 is right, a policy that starts and stays much more deterministic should let det-mode slip/progress track the clone tightly without the wide noisy-clone-sized loadslip band being a problem, since realized noise is now small everywhere. Prediction-if-true: det slip <=1.15x clone AND progress >=0.9x clone, zero falls, gait 6/6 -> PASS, launch rung B (heading-set respec). Prediction-if-false-collapsed-exploration: reward flat/falling from step 0 with high early falls or a frozen/degenerate gait (std too low to explore away from a still-imperfect warm start) -> the override is too aggressive, retry with a milder value (e.g. -1.5). Prediction-if-false-still-misses-cleanly: zero falls, gait_valid 6/6, low realized std confirmed in eval report, but slip/progress STILL miss the clone bars -> the sto/det-noise-band theory is refuted outright (not just weakly tested), DIG-IN required (per-leg gait video) before any further reward-mechanism edit.

**gate**: At 2M, DR-0, forward panel (eval_checkpoint det+sto with --cfg-set goal.walk_speed_min_m_s=0.08 goal.walk_speed_max_m_s=0.08 goal.walk_heading_max_rad=0.0) on the final checkpoint, clone-relative against the SAME control used for phasedir3/4 (ppo_goal_cw_bcgait_init_fullprof_phase1.zip, logs/ckpt_eval/phasedir3_clone_control_gate -- do not re-run it, same seeds/harness). CLONE-RELATIVE PASS ONLY: (a) zero falls, gait_valid 6/6; (b) along-command progress >= 0.9x clone; (c) slip/m <= 1.15x clone; (d) dir_err med <= clone + 5deg; (e) speed_mean in [0.06,0.096]; (f) report roll_tail_deg/drag_m vs clone. VERDICT MUST report policy_std from the eval report and state explicitly whether it actually dropped (target: well below 0.352, ideally <0.2) before drawing any conclusion. PASS -> pre-registered rung B respec: goal.walk_heading_set=[0,0.7854,-0.7854], same gate per heading bin. FAIL with std still not shrunk in eval (override didn't take / got overwhelmed by training) = escalate override magnitude or investigate why. FAIL with std confirmed low (<0.2) and gate still missed = the noise-band theory is REFUTED, DIG-IN required (per-leg gait metrics + video) before any further reward edit -- do not launch another anneal/override variant blind. FAIL with collapsed/degenerate behavior (frozen gait, early falls, reward cratering from step 0) = override too aggressive, retry milder (-1.5) as phasedir6, not a theory refutation. NO DOWNLOAD_ANSWER change from this run.

**verdict**: FAIL on the clone-relative gate (slip 1.590x clone, cap 1.15x) with the noise-band theory REFUTED per the pre-registered fork: std held at 0.13 (<0.2) all run, progress 0.984x clone (cap 0.9x, first arm ever to pass) and dir_err 0.794x/28.2deg PASS, zero falls, gait 6/6 -- yet det slip worsened monotonically across phasedir3/4/5 (1.41->1.518->1.59x) as std fell 0.365->0.13. DIG-IN root cause: MISALIGNED pricing, not exploration noise -- harness per-leg metrics show every phasedir arm swaps the clone high-cadence short-stride tripod (swing 0.255s, ~30 swings/leg, stride 0.0335m, duty 0.505-0.535) for a lower-cadence longer-stride duty-skewed gait (swing 0.335s, ~22 swings/leg, stride 0.042m, leg5 duty 0.57 vs tripod-A 0.46; swing_s-vs-slip r=0.75, zero overlap clone vs RL arms) that finances progress with dragged loaded feet; the training loadslip band (ok=7.0, sized to the std-0.368 clone noisy slip 4.8-6.4) NEVER FIRED after the std override (W&B rollout ratio 4.2-4.7, factor 0.99, excess -0.01/step) so slip was economically free while k_walk_course paid for the extra progress. Cheat encoded in bank (test_phasedir_* loadslip-band pins) + matched-env pod pricing A/B (logs/ckpt_eval/pd5_newband_ab_*); fix arm = band retighten (phasedir6), NOT another std/anneal variant.

