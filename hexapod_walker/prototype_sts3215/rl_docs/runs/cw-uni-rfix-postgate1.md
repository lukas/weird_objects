# cw-uni-rfix-postgate1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T15:13:28+00:00

**pod**: hexapod-mjx-train-0

**steps**: 18000000

**parent**: cw-uni-rfix-warm1 (init md5 78e5ef99; its lower 6/6 det posture-strict is the retention baseline)

**wandb_id**: 7d100irx

**hardware_ready**: False

**hypothesis**: The pre-registered next arm from the 08-10 14:58 dig-in cycle (posture-gated rise finish) plus the operator Stage-II landing: rfix-warm1 killed the paid-freeze plateau and got lower to 6/6 det posture-strict (uni-line first) but rise stays 0/12 by the NEXT unpriced hole -- at-height with legs 2 and 5 held 27-151mm aloft; height income has no foot-loading term. This arm: continue rfix-warm1 (same cfg, same DR0.5/15s/mix walk=0 hold=.1 rise=.45 lower=.45) + reward.rise_posture_gate=1 (landed cbf8a30: milestones, finish bonus and post-ramp kernel scale with the fraction of pads within end_posture_allow_m of grounded z -- geometric clearance matching the harness end_posture_ok, not touch force). ONE new variable vs the parent endpoint. If-true: det rise climbs off 0 on the posture-strict harness while lower 6/6 retains -- the income-hole ladder (freeze -> gameable-height) closes and the warm fine-tune-graft route stays open. If-false (rise still 0 with feet-aloft endings now unpaid): the blocker is exploration, not pricing -- next lever is the champion-reference tracking scaffold (reward.k_rise_ref_track + refs/rise_ref_stance_dr10.npz, same commit) which pays the KNOWN feet-down path densely. NOTE for triage: in-training rise fN/2 probes are the height-only criterion -- the posture gate may hold them lower while real rises form; judge on the harness + reward_rise_finish/rise_posture_factor curves, no flat-zero early call.

**gate**: posture-strict harness (default end-posture gate): rise >=5/6 det by 18M AND lower retains 6/6 det; VIDEO: no legs-aloft endings, no leg-through-floor

**verdict**: FAIL both gate clauses (rise 0/12, lower 0/12 posture-strict — lower was warm1's clean 6/6); dig-in found the gate's own pricing bug. OBSERVATIONS: lower det+sto 12/12 end with legs 3&5 held 33-109mm aloft as outriggers (duty 0.03-0.11) while h_err is excellent (0.1-6.6mm) and returns DOUBLE warm1's (466-532 vs 175-293); rise failure mode changed: bridge starts now reach height (h_err 1.2-2.7mm) with ONE leg at ~35mm (down from warm1's 27-151mm flag legs), flat/crouch starts trade height for posture (22-67mm short, 3 tilt_pitch terms); video: rigid outrigger leg through the whole lower descent, flag-leg dangle at rise finish. ROOT CAUSE (behavior<-incentive<-pricing, no sim defect): rise_posture_gate computes pf with end_posture_allow_m=20mm for ALL modes, but a belly-ending lower legitimately leaves pads 20-45mm up — measured on warm1's six harness-PASSING lowers (end clears 16.9-43.4mm), an HONEST lower earns pf 0.67-0.83 vs the cheat's 0.67; with no honest-vs-cheat differential, 18M more steps optimized warm1's incumbent mild spear-leg into a full outrigger that arrives faster and banks more post-ramp income. Rise side: the 1/6-per-leg linear pf leaves a shallow last-leg gradient (35->20mm worth ~17% of gated income vs a real balance cost — 3 tip-overs when it tries). Gate WAS live in training (warm1 proved the sim_env cfg path on MJX; _pad_z_ref in SNAP_ATTRS); rise_posture_factor absent from W&B was a PART_KEYS logging gap only. FIXES LANDED this cycle (cfg-gated default-off; smoke: legacy stream identical, honest lower pf 1.0; warm1-honest 16.9-43.4mm vs cheat 67-109mm now split by the 60mm allowance): pf uses the mode-correct allowance (end_posture_allow_lower_m=60mm for lower, matching harness + reward_end_posture) and rise_posture_factor/rise_income_factor added to trainer PART_KEYS. VERDICT: FAIL, checkpoint unusable, hardware-ready: no; warm1 remains the uni-line lower baseline. CAUTION: cw-stand-b2p1 (training) carries rise_posture_gate=1 on PRE-FIX code — its triage must check lower retention for this exact outrigger cheat. HYPOTHESIS STATUS: if-false branch confirmed with refinement — the fractional posture gate as implemented cannot close rise and actively broke lower; next lever is the dense reference-track scaffold already training (cw-stand-b2p1, k_rise_ref_track=2.0 + plant-height targets).

