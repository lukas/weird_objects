# cw-uni-rfix-postgate1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T15:13:28+00:00

**pod**: hexapod-mjx-train-0

**steps**: 18000000

**parent**: cw-uni-rfix-warm1 (init md5 78e5ef99; its lower 6/6 det posture-strict is the retention baseline)

**wandb_id**: 7d100irx

**hypothesis**: The pre-registered next arm from the 08-10 14:58 dig-in cycle (posture-gated rise finish) plus the operator Stage-II landing: rfix-warm1 killed the paid-freeze plateau and got lower to 6/6 det posture-strict (uni-line first) but rise stays 0/12 by the NEXT unpriced hole -- at-height with legs 2 and 5 held 27-151mm aloft; height income has no foot-loading term. This arm: continue rfix-warm1 (same cfg, same DR0.5/15s/mix walk=0 hold=.1 rise=.45 lower=.45) + reward.rise_posture_gate=1 (landed cbf8a30: milestones, finish bonus and post-ramp kernel scale with the fraction of pads within end_posture_allow_m of grounded z -- geometric clearance matching the harness end_posture_ok, not touch force). ONE new variable vs the parent endpoint. If-true: det rise climbs off 0 on the posture-strict harness while lower 6/6 retains -- the income-hole ladder (freeze -> gameable-height) closes and the warm fine-tune-graft route stays open. If-false (rise still 0 with feet-aloft endings now unpaid): the blocker is exploration, not pricing -- next lever is the champion-reference tracking scaffold (reward.k_rise_ref_track + refs/rise_ref_stance_dr10.npz, same commit) which pays the KNOWN feet-down path densely. NOTE for triage: in-training rise fN/2 probes are the height-only criterion -- the posture gate may hold them lower while real rises form; judge on the harness + reward_rise_finish/rise_posture_factor curves, no flat-zero early call.

**gate**: posture-strict harness (default end-posture gate): rise >=5/6 det by 18M AND lower retains 6/6 det; VIDEO: no legs-aloft endings, no leg-through-floor

