# cw-walk-step0-c2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T00:54:00+00:00

**pod**: hexapod-sweep-walk

**steps**: 4000000

**parent**: cw-walk-step0-c1

**wandb_id**: c1oyg4j6

**checkpoint**: rl_move/sim/policies/ppo_goal_cw_walk_step0_c2.zip

**hypothesis**: Operator directive 0-a continuation. c1 improved for real before a FALSE-POSITIVE canary auto-stop killed it at 1.26M/4M: canary protected rise_flat, a skill this walk-only from-scratch lineage never trained (parent baseline probe passed by accident); survived went 6/10 -> 10/10, rew 586 -> 591. c2 = identical config, init-from c1 final ckpt, --no-canary (nothing legitimate to protect; the walk gate harness is the real check). Predict: continued rise in ep_rew_mean and step-event metrics. If instead reward flattens or the gait degrades on video, the lineage has plateaued and stops per 0-a.

**gate**: Same as step0: DR 0, det AND sto, >=10cm forward, all six legs cycling lift/swing/touchdown, duty [0.2,0.9], >=2 swings/leg, no drag, no park; video pathology-first

**verdict**: FAIL — drag clause persists (det slip mean 0.818/0.865 across two 6-ep harness evals) PLUS new degradation: det tilt_pitch falls 2/12 (one on camera, walk_det_3.mp4 189f), policy_std 8.48, train/std runaway 3.24->8.73, rew flat (last0.5M 572 vs prev 578). NOT HARDWARE-READY. Hypothesis (continued improvement) REFUTED: identical-config ent-0.01 arm is actively destructive; lineage arm closed. A/B vs lowent decisive for entropy-runaway hypothesis.

**note**: IMPORTED 2026-08-09T01:20Z from the stale non-git copy at /workspace/prototype_sts3215 (operator ran the launcher from the wrong tree; this is the original launch record with checks/wandb_id). The cycle-written entry for the same run holds the verdict.

