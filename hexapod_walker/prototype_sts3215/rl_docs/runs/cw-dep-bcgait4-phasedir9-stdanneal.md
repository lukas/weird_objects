# cw-dep-bcgait4-phasedir9-stdanneal

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T11:55:44+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-dep-bcgait4-phasedir8-emakernel-allow24

**wandb_id**: mjeufod2

**hypothesis**: The robot keeps converging to a slow draggy gait because PPO's exploration noise makes honest walking look expensive to the reward — this arm gradually removes the noise during training so the reward's noise-free (measured-aligned) preferences take over. Mechanism: pd8 stack UNCHANGED + new --log-std-final -3.2 / --log-std-anneal-frac 0.6 (forced log_std schedule, std 0.135 -> 0.041 by 1.2M then held; the proven --warm-log-std-override mechanism on a schedule, on-pod verified) + reward.walk_course_overspeed_ref_floor_m_s=0.06 (new default-off cfg flooring the overspeed reference — insurance against the Warp-side ramp misfire pd8's W&B arithmetic exposed: mean charge -2.38/charged-tick vs mean exceedance 0.002 m/s). Measured basis (logs/ckpt_eval/pd8_digin_regime/, probe_stance_slip_dist --action-noise-std/--dr-scale): the honest clone at std 0.135 pays 0.76-9.7x its income in drag-stance charge at allow=24 (det: 0.002-0.36x; CPU replication reproduces the run's -2.87/tick to the decimal), and NO separating allowance exists (noisy-honest tail needs >=48mm while the pd6 det drag cheat pays ZERO beyond 36mm) — so det-calibrated per-stance/band pricing can never rank gaits correctly while the noise is present; the only coherent repair is converging the optimization regime to the det regime where full-stack pricing is measured-aligned (clone 1031 > pd7-slow 978 > pd6-drag 639). Prediction-if-true: as std falls, env/reward_drag_stance trends to ~0 and walk_loadslip_ratio toward the det clone band; det gate recovers progress >= 0.9x and speed >= 0.06 with slip <= 1.15x. Prediction-if-false-(i): std reaches 0.041 but drag charge stays large — the policy committed to a drag gait early and cannot unlearn it at low exploration; next arm anneals faster (frac 0.3), single change. Prediction-if-false-(ii): drag charge ~0 late AND gate still misses progress/speed — pricing is exonerated; the binding constraint is the BC-anchor/phase-lock family boundary; dig at anchor dose/phase_hz, NOT another reward edit. Prediction-if-false-(iii): the anneal destabilizes learning (ep_len collapses or falls appear late) — retry with a std floor of 0.08.

**gate**: At 2M, DR-0, same clone-relative forward panel and frozen control as pd8 (logs/ckpt_eval/phasedir3_clone_control_gate — do not re-run it). PASS requires ALL: (a) zero falls, gait_valid 6/6; (b) progress >= 0.9x clone; (c) slip/m <= 1.15x clone; (d) dir_err med <= clone + 5deg; (e) speed_mean in [0.06,0.096]. VERDICT MUST additionally report: final policy std (expect 0.041, log_std_anneal/frac 1.0 by 1.2M), env/reward_drag_stance across the anneal (expect trending to ~0 as std falls — if it stays large at low std, branch (i) fired), env/reward_walk_course_overspeed (expect ~0 all run under the ref floor; nonzero = the Warp ramp-misfire theory needs revisit), walk_loadslip_ratio trend vs std. 08-21 ruling: if the gate misses but drag charge is still falling with std at 2M, continue from checkpoint before re-specing. NO DOWNLOAD_ANSWER change from this run.

