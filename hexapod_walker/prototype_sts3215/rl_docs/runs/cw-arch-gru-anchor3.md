# cw-arch-gru-anchor3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-12T06:32:44+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-arch-gru-anchor2

**wandb_id**: k6ytmp3x

**hypothesis**: Fix the GRU's cross-mode walk freeze that survived turning off the walk-tick anchor. This arm tests whether the freeze comes from the demonstration-teaching's gradient bleeding into the SHARED recurrent trunk (the memory cell + input layers every mode's forward pass uses), not from teaching walking directly. Same exact recipe as cw-arch-gru-anchor2 (walk-tick anchor still off, rise/hold/lower anchored, same warm start from the ft1 BC-distilled base) with ONE new variable: train.bc_anchor_detach_trunk=1 (new CODE, landed 08-12, default off/bit-exact, tests green in test_gru_policy.py) stops the anchor's gradient at the GRU/feature-extractor output so stance-tick anchoring only trains the actor head (mlp_extractor.forward_actor + action_net), never the shared trunk. Prediction-if-true: walk recovers (det gait_valid>=5/6, prog_ratio med>=0.80, real translation on video) while hold/lower keep anchor2's levels. Prediction-if-false: walk still freezes even with the trunk protected -- meaning the interference is not the anchor's gradient at all (likely the main PPO loss's own shared-trunk credit assignment across mode ticks), the anchor-for-recurrent-nets line closes for good, and the remaining lever is a mode-gated/separate recurrent core (architecture work, not a training-loss tweak).

**gate**: PASS if det walk gait_valid>=5/6 AND prog_ratio med>=0.80 (no freeze/paddle, video confirms real translation, not just a passing metric) AND hold det>=4/6 AND lower det>=4/6. FAIL if walk still freezes/paddles (detach_trunk doesn't fix it -> the anchor-for-recurrent-nets line closes for good) OR hold/lower drop below 4/6 (detaching the trunk broke the actor-head-only training path).

