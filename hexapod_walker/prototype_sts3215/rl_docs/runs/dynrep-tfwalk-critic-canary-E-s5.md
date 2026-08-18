# dynrep-tfwalk-critic-canary-E-s5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-17T05:39:37+00:00

**pod**: hexapod-mjx-train-11

**steps**: 100000

**git_sha**: cc8c144f5032e4e17765c7c7c068549b1b48969c

**wandb_id**: x2lgwif4

**hypothesis**: Integration canary for the decoupled predictive-critic idea: the robot walking policy itself stays a plain from-scratch PPO actor, while ONLY its value function (critic) gets to read a continuously-learning dynamics world model through a zero-initialized gate. This 100k-step canary does not test learning quality - it mechanically proves the wiring: CUDA active, actor bit-independent of the predictor (zero action-KL proof metric), snapshot transformer frozen across each rollout+PPO iteration with guarded EMA updates only between iterations, W&B metrics advancing, checkpoints ~60MB not 12GB.

**gate**: PASS = all hard requirements hold over the full 100k: (1) [device] CUDA line before W&B; (2) pred/actor_kl_from_predictor exactly 0.0 on every iteration and no actor-mutation RuntimeError; (3) snapshot version stable within iterations (no mutation raise), pred/snapshot_version advancing only via accepted EMA updates with drift-guard accounting logged; (4) run-end zip saves ~60MB (no runtime pickled), reloadable; (5) rollout/eval/pred/critic W&B channels present and advancing. Any assertion failure = STOP and fix; do not scale to the 1M cohort.

**verdict**: PASS - all pre-registered canary hard gates hold over the full 100k on CUDA: actor_kl_from_predictor exactly 0 on all 49 iterations (no mutation raise), snapshot version monotone 0->49 via accepted EMA only (0 rejections, drift_step 2.6e-5 vs 0.05 guard), saves 58.8MB (+54.5MB separate online predictor) and reload verified on-pod, all rollout/eval/pred/critic W&B channels advancing. Bonus signal: online-predictor heldout 2.509 vs pretrained 2.286 (+9.7%, inside the 15% band), gate learned to -0.20 with residual contributing. Cleared to launch the D/E seeds-5/6/7 1M cohort.

**note**: Script-owned (pod_tfwalk_critic.sh, cohort tfwalk-critic-canary). Operator directive fb_20260817T052333_e5ae09: <=100k one-seed E integration canary, hard-gated before the D/E seeds-5/6/7 1M cohort. Code exp/cw-dynrep-tfwalk-critic1 (cc8c144f), tests 8/8 new bank + all dynrep banks green.

