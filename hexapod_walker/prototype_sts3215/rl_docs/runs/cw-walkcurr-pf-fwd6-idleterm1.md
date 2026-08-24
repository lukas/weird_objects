# cw-walkcurr-pf-fwd6-idleterm1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-24T02:21:11+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-rscale50

**wandb_id**: xadaecty

**hypothesis**: Plain English: does ending the episode early (with a modest dedicated penalty) the instant the robot's joints go essentially motionless for a few seconds finally break the static-crouch/park-stand local optimum that every PRICED-ONLY anti-park lever (idle charge, park_duty up to 1.5x dose, height-gate loose+tight, RND dose 0.02-1.0+budget) failed to dislodge across 9 independently-refuted mechanism classes? Single mechanism addition on the exact fwd6-rscale50 recipe (x0.02-scaled reward stack, fresh 2M discovery, no warm start): safety.walk_idle_terminate_s=3.0 (evict after 3s of near-zero mean |joint velocity| across the 18 actuated joints, past a 3s settle grace) + reward.walk_idle_terminate_penalty=3.0 (x0.02-scaled dedicated penalty, distinct from term_penalty=24, bank-tuned so a captured park does not invert the park<sideways ordering) + qvel floor 2 deg/s. Bank-proven in test_task_semantics.py (test_walkcurr_idle_term_*, 4/4 green): mean |qvel| is a surgical discriminator -- only a literally frozen policy output (~5e-5 rad/s) gets captured; gait/stall/reverse/sideways/skate/topple (all >=0.1 rad/s, genuinely commanded motion) are bit-exact unchanged, so the mechanism cannot reopen the 'wrong-way beats standing still' or 'skate escapes its slip charge' failure modes a naive body-speed or along-command-speed version was measured to cause. Prediction-if-true: env/walk_freeprog_score crosses the [-0.10,-0.05] dead band every one of the 9 refuted mechanisms sat in, and/or the det gate panel shows real stepping on video (gait_valid trending up, height_err_end_mm leaving the ~116mm belly-sit signature). Prediction-if-false (identical belly-sit/frozen-crouch signature persists with a healthy optimizer): idle-termination becomes the 10th refuted mechanism and BC-kickstart is the only unexplored escalation left in the track's own pinned fork order. Strongest alternative: the termination fires as designed (mean episode length collapses toward ~6s, the grace+idle_terminate_s window) but the policy simply re-learns to be motionless for just under 6s per episode rather than discovering real walking -- watch mean episode length alongside freeprog, not gate pass/fail alone.

**gate**: Rung-1 gate (same as sibling fwd6 arms): C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes. Mechanism-health check (own-cfg wandb read, not just the gate pass/fail): env/walk_freeprog_score trend vs the [-0.10,-0.05] band every prior rung-1 arm sat in; mean episode length should NOT sit flat at ~6s (grace+idle_terminate_s) for the whole run, which would mean the policy learned to survive just past the boundary rather than to walk; clip_fraction stays healthy (crush-fix precedent).

