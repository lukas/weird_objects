# cw-standwalk-stance-mesh2-refgain15

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T07:03:29+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-standwalk-stance-mesh2-cur1

**wandb_id**: 75dqyaxt

**hypothesis**: Plain English: cur1's own pricing (current_hot+term_cost) removed the profitable grind exploit but PPO never found any honest stable basin either (reward flat all 20M, 0/3 seeds pass). Candidate cause: the dense guidance toward the KNOWN-GOOD rise trajectory (k_rise_ref_track=2.0, unchanged since the primitive era) is too weak to out-compete the other shaping terms once the cheap grind path is priced out, so exploration never gets pulled onto the reference path before falling. This arm raises k_rise_ref_track 2.0->15.0 (7.5x), single lever, same pricing/goal-mix/budget as cur1. Prediction-if-true: rise panel starts landing valid plants (>=5/6) by 20M where cur1 landed 0/6; reward trend breaks out of the flat negative band cur1/seed1/seed2 all showed. Prediction-if-false: reward stays flat/negative -- ref-tracking gain was not the bottleneck, escalate to the physics/gains audit (log-std-init/ent-coef) or budget-ladder fork named in STATUS.md instead.

**gate**: Same stage-1 gate as cur1: pod_eval stance panel (rise/hold/lower) n>=12 det+sto DR-0+own-DR(0.2) at 20M: rise ends valid plant >=5/6 det AND sto, lower posture-strict >=5/6, hold quiet 6/6, no crash-lowering on video. FAIL if reward stays in cur1's flat/negative band or any mode stays at 0/6.

