# cw-dep-bcgait4-phasedir4-entanneal

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-22T06:08:44+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-dep-bcgait4-phasedir3-fwd-reprice

**wandb_id**: n6zpuwvc

**hardware_ready**: False

**hypothesis**: Same fwd-only obedience task and repriced loadslip/overspeed reward as phasedir3-fwd-reprice, but let PPOs action noise actually shrink during training instead of staying pegged at std~0.355-0.365 the whole run: anneal ent_coef from 0.001 down to 0.0001 over the first 60% of steps (new default-off train_ppo_mjx --ent-coef-final/--ent-coef-anneal-frac, built+on-pod-smoke-tested this cycle). phasedir3 proved the loadslip repricing itself works as intended (factor never zeroed, det speed matched clone) but its band -- sized to spare the stochastic clones noisy slip (4.8-6.4) -- also spared a drifted deterministic mean once std stays wide forever, so det slip drifted to 1.41x clone (cap 1.15x) while progress reached 0.897x clone (cap 0.9x, narrowly short). If std actually shrinks, the same flat threshold should stop being loose for det-mode and progress/slip should both clear the clone-relative bars without any further reward-mechanism change. Prediction-if-true: det slip <=1.15x clone AND progress >=0.9x clone, zero falls, gait 6/6 -> PASS, launch rung B (heading-set respec). Prediction-if-false-weak-anneal: eval std still close to phasedir3s 0.355-0.365 (the anneal barely moved it, e.g. optimizer momentum or the BC anchor keeps forcing variance) -> escalate ent-coef-final lower / anneal-frac earlier, not a new charge. Prediction-if-false-anneal-worked-anyway: std demonstrably shrunk (e.g. <0.2) but slip/progress still miss the clone bars -> the sto/det-noise-band theory is wrong and this needs a full dig-in (per-leg gait metrics, video) before any further reward edit.

**gate**: At 2M, DR-0, forward panel (eval_checkpoint det+sto with --cfg-set goal.walk_speed_min_m_s=0.08 goal.walk_speed_max_m_s=0.08 goal.walk_heading_max_rad=0.0) on BOTH the final checkpoint AND the matched clone control ppo_goal_cw_bcgait_init_fullprof_phase1.zip (same seeds/harness) -- identical gate to phasedir3-fwd-reprice. CLONE-RELATIVE PASS ONLY: (a) zero falls, gait_valid 6/6; (b) along-command progress >= 0.9x clone; (c) slip/m <= 1.15x clone; (d) dir_err med <= clone + 5deg; (e) speed_mean in [0.06,0.096]; (f) report roll_tail_deg/drag_m vs clone. VERDICT MUST report the trained policys final action std (policy_std in the eval report) alongside the gate table -- if std did not meaningfully shrink vs phasedir3s 0.355-0.365, say so explicitly before drawing any conclusion about the noise-band theory. PASS -> pre-registered rung B respec: goal.walk_heading_set=[0,0.7854,-0.7854], same gate per heading bin. FAIL with std unchanged = anneal too weak, retune ent-coef-final/frac, not a new charge. FAIL with std shrunk and gate still missed = the noise-band theory is refuted, DIG-IN required before any further reward edit. NO DOWNLOAD_ANSWER change from this run.

**verdict**: FAIL (regressed vs phasedir3): ent-coef anneal confirmed WORKING at the ent_coef level (0.000951->0.0001, verified via wandb ent_coef_anneal/value) but policy_std barely moved (0.368->0.352, clone/phasedir3 both ~0.355-0.368) -- entropy bonus is too small a loss fraction to drag a warm-started log_std down in 2M steps. Clone-relative: progress 0.830x (cap 0.9x, WORSE than phasedir3 0.897x), slip 1.518x (cap 1.15x, WORSE than phasedir3 1.41x); zero falls, gait 6/6, dir_err/speed both pass. Matches the pre-registered weak-anneal branch exactly.

