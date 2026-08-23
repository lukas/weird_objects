# cw-walkcurr-pf-rung0-swing3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T21:43:29+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**wandb_id**: icv9nggt

**hypothesis**: Plain English: ten prior-free arms froze into a motionless crouch because every reward for moving required already knowing how to walk -- this run changes the goal itself to 'lift your feet and step, anywhere' so a random policy can stumble into income within one flail. Rung-0 diet at the crush-proven x0.02 scale: k_walk_swing=0.06 pays any completed >=15mm swing (the only income reachable from random flail), travel-demanding charges REMOVED (heading=0, idle_charge=0, step_event=0), freeprog kept at 10% dose (0.006: keeps the legacy speed kernel replaced + slight forward tilt), park_duty/loadslip/term kept so freezing, skating, dying stay priced. Bank test_walkcurr_rung0_* 15/15 GREEN at this exact cfg (snapshot exp/walkcurr-rung0-bank): gait +479 > creep +256 > stall +38 > shuffle -185 > stork -320 ~ park -324 > topple -1164 > skate -1311 at x1; deltas linear at x0.02. Prediction-if-true: env/reward_swing per-step trends up from ~0 within 2M, det panel shows six legs cycling (gait_valid >=4/6) even without travel; PASS -> rung-1b warm-start on the rung-1 diet. Prediction-if-false (frozen crouch again, reward_swing flat ~0 with healthy clip_fraction): the reward-landscape theory is refuted alongside the optimizer theory -- escalate to RND state-novelty per track STATUS. Strongest alternative: policy farms swing income by shuffling forever and rung-1b later fails to convert -- acceptable for rung-0 (cycling IS the certified goal; the bank keeps walking strictly on top so the gradient continues forward).

**gate**: Rung-0 certification gate: C-env det fixed-forward panel -- zero tilt terms, six legs cycling (gait_valid) on >=4/6 det episodes, video shows rhythmic stepping (in place or travelling); travel/prog NOT required, slip not gated (priced by loadslip charge). Mechanism health at 2M: env/reward_swing per-step > 0 and rising, clip_fraction > 0.02. PASS = certify + launch rung-1b warm-start (rung-1 rscale50 diet from this checkpoint). FAIL (static crouch, reward_swing ~0) on BOTH dose arms = rung-0 refuted -> RND escalation, no dose resweep.

