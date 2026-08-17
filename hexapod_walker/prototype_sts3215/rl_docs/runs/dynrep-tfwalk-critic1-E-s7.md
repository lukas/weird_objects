# dynrep-tfwalk-critic1-E-s7

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-17T05:47:19+00:00

**pod**: hexapod-mjx-train-5

**steps**: 1000000

**git_sha**: cc8c144f5032e4e17765c7c7c068549b1b48969c

**wandb_id**: xs3fgez1

**hypothesis**: Give the walking robot a better value function without touching how it acts: the actor is the plain from-scratch PPO policy (bit-identical init to scratch A), and only the CRITIC reads a dynamics transformer that KEEPS LEARNING from the robot own walking experience (own optimizer, fresh rollout windows + 25% v5 rehearsal), delivered through a drift-guarded EMA snapshot updated only between rollout+PPO iterations. Tests whether a continually-updated world model helps PPO as critic-only input (E = online arm of the D/E pair).

**gate**: Pre-registered 1M decision checkpoint, NO extension without a verdict. PASS only if E: (1) preserves online-predictor heldout prediction quality within 15% of the pretrained 2.286 reference; (2) keeps actor approx-KL comparable to scratch A (~0.02, no late regression) with pred/actor_kl_from_predictor exactly 0 throughout; (3) improves critic explained-variance/sample-efficiency AND heldout walking over BOTH scratch A and frozen D across seeds, without gait regression (slip_m, peak_roll_deg, slew_sat vs A). If E does not beat D and A, record that predictive representation does not help PPO here and STOP this line. Retain best-by-heldout checkpoints, not merely final.

**note**: Script-owned cohort (pod_tfwalk_critic.sh, manifest tfwalk-critic1_manifest.jsonl on-pod). Operator directive fb_20260817T052333_e5ae09 (decoupled predictive-critic transfer); code exp/cw-dynrep-tfwalk-critic1 (cc8c144f), 8/8 new bank + all dynrep banks green, E integration canary PASSED all hard gates (dynrep-tfwalk-critic-canary-E-s5). Controls = existing config-equivalent scratch A: joint1 A-s6 (c58ft8qp) / A-s7 (3qqitw5h) at 1M + metrics1 A-s5 (jf0tfsqh) at its 1M eval point - core MDP/PPO config keys mechanically verified identical vs W&B configs this cycle. Prior B/C are NOT controls (they changed actor inputs).

