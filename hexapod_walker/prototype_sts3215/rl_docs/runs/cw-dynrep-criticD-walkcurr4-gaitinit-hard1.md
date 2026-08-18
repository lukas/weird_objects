# cw-dynrep-criticD-walkcurr4-gaitinit-hard1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-18T10:26:56+00:00

**pod**: hexapod-mjx-train-7

**steps**: 4000000

**parent**: cw-dynrep-criticD-walkcurr4-canA-r1

**wandb_id**: jzegb7tg

**hypothesis**: Can actor-only initialization from the RL-HARDENED tall-walk checkpoint ppo_goal_cw_dep_bcgait1_hard1 (10M fine-tune on top of the same BC clone: height -8.5..-9.8mm, slip 1.27-1.54/m, zero falls, the campaign's strongest tall-walk checkpoint) beat scratch acquisition under the SAME B0 gates + LR recipe as arm A (cw-dynrep-criticD-walkcurr4-canA-r1/-r2, seed 8, actor-lr 2e-4 x5 epochs held to B0 promotion)? Operator addendum fb_20260818T085834_588d9a (filed ~08:58:34 UTC, correcting fb_20260818T085648_2a0a60 AFTER canA-r1/canB-r1 were already in flight): height/progress gates alone are a CLOSED lever (cw-dep-hgt1/hgt2 stayed -52..-77mm); the positive mechanism in the campaign is actor initialization from the scripted tripod gait, not the acquisition LR. NEW CODE this cycle: --init-from-actor-only (train_ppo_mjx.py + predictive_critic.actor_only_transplant, default off, unit-tested test_actor_only_transplant.py 9/9): builds the FRESH condition-D model (fresh critic, fresh zero-gated predictive residual, scratch-A actor shape) then copies ONLY the actor tensors from --init-from, zero-padding the policy trunk's first-layer columns across the obs widening (single frame -> hist16-stacked, newest-first) so the transplanted actor reproduces the source policy bit-for-bit at init regardless of the older-history tail; critic/frozen-encoder NEVER read from --init-from (asserted by the test bank). CUDA canary canary-walkcurr4-gaitinit-hard1 (train-5, this exact CLI, killed early at ~40k steps for an accidental GPU-sharing collision with a concurrent cycle's canC-r1 landing on the same pod -- not a code failure; the identical code path is already canary-proven end-to-end by the sibling gaitinit-bcinit run (train-11, 600k steps: backend VERIFIED, transplant confirmed 7 actor tensors copied, clean training approx_kl~0.007/zero rollbacks/ep_rew_mean 112->138) since both checkpoints share the exact same architecture/obs contract (net_arch [128,128], obs 72, act 18) that the transplant code keys off. NAMED DISTINCTLY (not reusing the letters 'B'/'C') because a concurrent cycle already launched canB-r1/canC-r1 under the PRE-addendum design (scratch + LR sweep) before this addendum was read; those runs remain valid LR-sensitivity controls, not mistakes -- see dynrep/STATUS.md reconciliation note. Prediction-if-true: B0 certs upright (height_factor>=0.8, cmd_prog_frac>=0.5) much earlier than the scratch actors on canA/canB-r1/canC-r1, since the init already walks tall and in-band; if-false: the actor-only transplant still gets erased by the frozen-critic-D optimizer dynamics into the same crouch-shuffle. Strongest alternative: gait-init survives ignition but the condition-D critic's income still argues for the crouch at higher buckets (informative either way).

**gate**: Behavioral admission at 4M, judged on B0 cert telemetry (NOT crash-free): cmd_prog_frac>=0.50, height_factor>=0.8, falls==0 in final cert round, slip_per_m<=3.0, positive recent B0 progress slope; B0 promotion preferred. Compare against canA/canB-r1/canC-r1 (scratch actors, same gates) on retention-clean frontier reached, THEN cmd_prog_frac, slip, roll, and TIME-TO-B0-PROMOTION (the actor-init hypothesis predicts much faster ignition). Winner across ALL SIX canaries contributes its RECIPE to the 40M cw-dynrep-criticD-walkcurr4; never launch the 40M on a failing recipe.

