# cw-standwalk-stance-mesh2-cur1-seed2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T06:00:15+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-standwalk-stance-mesh1-rr1

**wandb_id**: yeef6cxg

**hypothesis**: Seed 2 of the cw-standwalk-stance-mesh2-cur1 3-seed batch (see seed-0 entry for the full plain-English rationale + probe evidence): mesh/100Hz from-scratch stance recipe with the probe-chosen near-saturation current price (k_current_hot=1.0 @ 2.0 A) + termination horizon cost (term3 cap 60). The mesh1 predecessor at this seed slot (seed 2) was the WORST failure (36/36 term, tilt-dominated: real 2/3-height rises then lateral tips) — watch whether pricing alone also stabilizes the mid-rise balance or only removes the grind. Distinct-seed robustness arm; same predictions as seed 0.

**gate**: Read jointly with cw-standwalk-stance-mesh2-cur1 (seed 0) at 20M: stance panel rise/hold/lower n>=12 det+sto DR-0 + own-DR(0.2), zero falls/tips, rise valid plant >=5/6, lower posture-strict >=5/6, hold quiet 6/6. 2-3/3 healthy = recipe robust; 0-1/3 = named fork, not more seeds.

**verdict**: Seed replicate of the cur1 batch, JOINT FAIL 0/3 (see cw-standwalk-stance-mesh2-cur1 for the full joint writeup). This seed: 35/36 stance episodes terminated (hold/rise/lower, det+sto, DR-0), term signature tilt_roll-dominant (18) + over_current (9) + tilt_pitch (8). Video (hold_det): starts in a passable compact tripod-like stance, holds briefly, then progressively rolls/slides toward the end of the episode into a tilt_roll termination. Reward quarters -315/-516/-298/-364 (dip then ends WORSE than it started -- no learning progress, matches the 08-21 'genuine FAIL' bar). Confirms 0/3: the realigned pricing fix stops the saturation-grind cheat but this seed also cannot hold stance through a full episode in any mode. Next: rung-2 escalation as recorded on the base seed, not more same-recipe seeds. Evidence: logs/ckpt_eval/cw_standwalk_stance_mesh2_cur1_seed2_gate/report.json, W&B yeef6cxg.

