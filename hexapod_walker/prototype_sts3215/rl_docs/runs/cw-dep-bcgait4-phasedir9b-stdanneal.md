# cw-dep-bcgait4-phasedir9b-stdanneal

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-22T11:49:40+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-dep-bcgait4-phasedir8-emakernel-allow24

**hypothesis**: Phasedir8 kept the pd7 fix (allow=24mm drag charge, EMA velocity kernel) but still missed the gate (progress 0.770x, slip 1.254x, speed 0.0575) because the dig-in found the det-calibrated pricing simply does not transfer to the noisy PPO rollout regime: at the training action-noise std of 0.135 the honest clone's own per-stance travel tail needs an allowance >=48mm to stay untaxed, while the det pd6 drag-cheat pays zero drag charge out past 36mm -- no single allowance value can separate honest from cheat while std stays high. This run keeps the exact pd8 reward stack unchanged and instead anneals the POLICY's action noise itself from std=0.135 down to std=0.04 over the first 60% of training (the already-proven --warm-log-std-override mechanism, now scheduled) plus floors the overspeed reference (reward.walk_course_overspeed_ref_floor_m_s=0.06) to kill a secondary Warp-only bug where command-ramp ticks at near-zero s_ref paid the full overspeed clip on 2mm/s of drift. Prediction-if-true: as std falls toward the det regime, W&B env/reward_drag_stance decays toward ~0 by the 60% mark and the final det-mode gate numbers converge to phasedir8's OWN det numbers or better (progress>=0.9x, slip<=1.15x, speed in [0.06,0.096]) because the measured-aligned det pricing (clone 1031 > pd7-slow 978 > pd6-drag 639) becomes the actual operating point instead of an unreachable aspiration. Prediction-if-false-(i): drag charge does NOT fall to ~0 even once std<0.05 late in training -- the policy has learned a genuinely slow/dragging gait that is not explained by noise at all, and pricing is exonerated; the binding constraint is the BC-anchor/phase-lock family boundary (this lineage's floor), not the reward -- dig there next. Prediction-if-false-(ii): drag charge falls but progress/speed do NOT recover -- annealing noise froze the policy in the SAME degenerate optimum it was already in before it could explore back out; would need to anneal from an earlier/less-converged checkpoint or add explicit exploration pressure back after the anneal. Prediction-if-false-(iii): speed overshoots past 0.096 or hits the known 0.139 attractor once noise is low enough for k_walk_course to bind cleanly -- the ref-floor or k_walk_prog=2 needs a small pull-back, single change.

**gate**: At 2M steps (matched to phasedir8), DR-0 forward panel, same eval_checkpoint det+sto invocation as phasedir8's gate, clone-relative against the SAME frozen control (logs/ckpt_eval/phasedir3_clone_control_gate -- do not re-run it). PASS requires ALL: (a) zero falls, gait_valid 6/6; (b) progress >= 0.9x clone; (c) slip/m <= 1.15x clone; (d) dir_err med <= clone + 5deg; (e) speed_mean in [0.06,0.096]. VERDICT MUST additionally report: the log_std_anneal/std W&B trajectory (expect 0.135 -> ~0.04 by 60% of steps, held after), env/reward_drag_stance trajectory (expect trending to ~0 as std falls, not flat-negative like phasedir8), and env/walk_course_overspeed_m_s charged-tick rate (expect the tiny-s_ref ramp-drift class to disappear). Per the 08-21 ruling: if gate misses but reward AND gate metrics (speed, slip ratio) are still improving at 2M, continue from checkpoint before re-specing. FAIL WITH std confirmed annealed near-det (<0.05) AND drag charge NOT falling to ~0 late means pricing is exonerated -- next dig-in targets the BC-anchor/phase-lock family boundary directly, not the reward stack. NO DOWNLOAD_ANSWER change from this run.

**refused_reason**: hexapod-mjx-train-0 code marker 8a391c947a4f7427db42d0a728e015a135ea1edd != local HEAD 599d0c83ad6ee0f3f79ff096e6c818d01a91e56c. Sync first: snapshot.sh --sync hexapod-mjx-train-0 (and snapshot/commit before that if the tree is dirty).

