# cw-walkcurr-pf-fwd6-rscale50-gru

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-24T02:41:45+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-rscale50

**wandb_id**: qhpnc88y

**hypothesis**: Plain English: the original --gru arm (fwd6-gru) and the original --use-sde arm (fwd6-sde) were BOTH tested at the raw unscaled v2e reward dose (term_penalty=1200 etc.), i.e. before the optimizer-crush root cause and its rscale50 (x0.02) fix were found -- their own W&B histories show the identical clip_fraction-collapses-to-0 signature every memoryless-MLP arm showed at that dose, so recurrence was never actually given a healthy optimizer to work with. This is the pre-registered cross-test STATUS.md itself named ('if rscale FIXES the freeze, re-test gSDE/GRU restricted to whether they change anything ADDITIONAL on top of the rescaled reward') and it has not been run. Single lever vs fwd6-rscale50: swap the memoryless MLP actor/critic for sb3-contrib RecurrentPPO (--gru, --n-steps=64 -- the documented BPTT window for --gru, replacing rscale50's --n-steps=24) on the EXACT x0.02-scaled crush-fixed recipe, fresh 2M discovery, no warm start. Prediction-if-true: with clip_fraction now healthy AND memory available, env/walk_freeprog_score crosses zero and/or det gate shows real stepping -- recurrence adds something the crush-fixed MLP (rscale50/rscale50-cont1, both closed flat) could not. Prediction-if-false (identical belly-sit signature, healthy clip_fraction): recurrence is cleanly exonerated even with a healthy optimizer, closing this STATUS.md-pinned gap and adding to the now-10-strong refuted-mechanism tally without qualification (every prior GRU finding was confounded by the crush; this one is not).

**gate**: Same rung-1 gate as every fwd6 arm: C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes. Mechanism-health: clip_fraction stays healthy (no exact-0 collapse, the crush signature); env/walk_freeprog_score trend vs the [-0.10,-0.05] band.

