# cw-standwalk-stance-mesh2-cur1-seed1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T05:58:06+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-standwalk-stance-mesh1-rr1

**wandb_id**: h8nqxuk2

**hypothesis**: Seed 1 of the cw-standwalk-stance-mesh2-cur1 3-seed batch (see that entry for the full plain-English rationale + probe evidence): mesh/100Hz from-scratch stance recipe with the probe-chosen near-saturation current price (k_current_hot=1.0 @ 2.0 A) + termination horizon cost (term3 cap 60) that flips the measured ground-grind exploit negative in every mode while honest behavior keeps 90%+ of its return. Distinct-seed robustness arm; same predictions as seed 0.

**gate**: Read jointly with cw-standwalk-stance-mesh2-cur1 (seed 0) at 20M: stance panel rise/hold/lower n>=12 det+sto DR-0 + own-DR(0.2), zero falls/tips, rise valid plant >=5/6, lower posture-strict >=5/6, hold quiet 6/6. 2-3/3 healthy = recipe robust; 0-1/3 = named fork, not more seeds.

**verdict**: Seed replicate of the cur1 batch, JOINT FAIL 0/3 (see cw-standwalk-stance-mesh2-cur1 for the full joint writeup). This seed: 35/36 stance episodes terminated (hold/rise/lower, det+sto, DR-0), term signature tilt_roll-dominant (14) + over_current (13) + tilt_pitch (8). Video (rise_det): starts already collapsed/press-up (front nose to ground, legs splayed sideways), never reaches a valid plant across the clip -- same sprawled-press-up signature as seed0. Reward quarters -318/-533/-311/-297 (dip-and-partial-recover, no real trend). Confirms 0/3 (not a single-seed fluke): the realigned mesh2/cur1 pricing fix closes the saturation-grind exploit but the honest attempt still topples in every mode at 20M steps. Next: rung-2 escalation as recorded on the base seed (budget/curriculum/balance-shaping), not more same-recipe seeds. Evidence: logs/ckpt_eval/cw_standwalk_stance_mesh2_cur1_seed1_gate/report.json, W&B h8nqxuk2.

