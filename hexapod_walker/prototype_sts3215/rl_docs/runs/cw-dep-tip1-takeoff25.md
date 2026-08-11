# cw-dep-tip1-takeoff25

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-11T23:09:22+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-dep-tip1

**wandb_id**: x2p1vnnp

**hypothesis**: HARDWARE-DRIVEN (operator bench 08-11 eve, camera sessions bench_blast_20260811_18*, RL_LOG 22:47): BOTH deployed walkers show a 20-25deg TAKEOFF roll transient right after gait start on hardware — vref1-r1 recovered one (23deg peak -> tail 0.9deg, full 6s) and fell one; tip1 tripped tilt_roll 2/3 fwd (attended 21:4x, robot's own event log) and fell its first rot60 BACKWARD run (27deg peak). Trained tipped-start DR is dose 6-18deg at prob 0.30 — BELOW the measured regime. Arm: warm from tip1, raise dr.tipped_start_prob to 0.5 and dr.tipped_start_deg to (12,25), everything else = tip1's exact resolved config (pull from its W&B config.reward_cfg / ledger extra_args). NOTE: verify --cfg-set can parse the tuple field dr.tipped_start_deg; if not, add the knob first (small domain_rand.py change, keep the guarded-rng convention + bank green).

**gate**: SCORE/tipped_recovery_success at 20-25deg injections det+sto materially above the matched parent under the IDENTICAL injection (eval_checkpoint --baseline cw-dep-tip1); DR0 walk retention unchanged (gait_valid 6/6, prog med >= 0.85, no paddle); frames watched det.

