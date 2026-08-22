# cw-dep-bcgait4-phasedir9b-stdanneal

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T11:53:48+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-dep-bcgait4-phasedir8-emakernel-allow24

**wandb_id**: t0mgnv2x

**hypothesis**: Phasedir8 kept the pd7 fix (allow=24mm drag charge, EMA velocity kernel) but still missed the gate (progress 0.770x, slip 1.254x, speed 0.0575) because the dig-in found the det-calibrated pricing simply does not transfer to the noisy PPO rollout regime: at the training action-noise std of 0.135 the honest clone's own per-stance travel tail needs an allowance >=48mm to stay untaxed, while the det pd6 drag-cheat pays zero drag charge out past 36mm -- no single allowance value can separate honest from cheat while std stays high. This run keeps the exact pd8 reward stack unchanged and instead anneals the POLICY's action noise itself from std=0.135 down to std=0.04 over the first 60% of training (the already-proven --warm-log-std-override mechanism, now scheduled) plus floors the overspeed reference (reward.walk_course_overspeed_ref_floor_m_s=0.06). Prediction-if-true: drag charge decays toward ~0 by the 60% mark and final det gate numbers converge to/beat phasedir8's own (progress>=0.9x, slip<=1.15x, speed in [0.06,0.096]). Prediction-if-false-(i): drag charge does NOT fall to ~0 even once std<0.05 late -- pricing is exonerated, the binding constraint is the BC-anchor/phase-lock family boundary, dig there next. Prediction-if-false-(ii): drag charge falls but progress/speed do NOT recover -- annealing froze the policy in the same degenerate optimum; would need to anneal from an earlier/less-converged checkpoint. Prediction-if-false-(iii): speed overshoots past 0.096 or hits the known 0.139 attractor -- ref-floor or k_walk_prog=2 needs a small pull-back.

**gate**: At 2M steps (matched to phasedir8), DR-0 forward panel, same eval_checkpoint det+sto invocation as phasedir8's gate, clone-relative against the SAME frozen control (logs/ckpt_eval/phasedir3_clone_control_gate). PASS requires ALL: (a) zero falls, gait_valid 6/6; (b) progress >= 0.9x clone; (c) slip/m <= 1.15x clone; (d) dir_err med <= clone + 5deg; (e) speed_mean in [0.06,0.096]. VERDICT MUST additionally report log_std_anneal/std trajectory (expect 0.135->~0.04 by 60%), env/reward_drag_stance trajectory (expect ~0 as std falls), and overspeed charged-tick rate. Per 08-21 ruling: gate miss + reward/gate metrics still improving = continue before respec. FAIL WITH std annealed near-det AND drag charge NOT falling to ~0 = pricing exonerated, dig the BC-anchor/phase-lock boundary next, not the reward. NO DOWNLOAD_ANSWER change from this run.

