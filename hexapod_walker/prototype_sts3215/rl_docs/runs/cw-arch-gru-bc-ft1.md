# cw-arch-gru-bc-ft1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-11T19:51:28+00:00

**pod**: hexapod-mjx-train-1

**steps**: 10000000

**wandb_id**: iv0ohvk5

**hypothesis**: gru-r1..r4c post-mortem: the from-scratch GRU rung was budget-confounded (hist16-r7, the MLP reference, needed 40M walk-only steps; the GRU runs got 2M mixed) — from-scratch exploration finds the paddle first at discovery budget, but nothing is wrong with GRU+PPO itself (test_gru_policy.py incl. memory-task learning, all pass). So skip exploration: BC-distill both champions into one GRU (walks + holds out of the box), then PPO fine-tune with the anti-cheat stack on the mixed diet to lift rise/lower and polish. If true, one recurrent policy walks AND sits/stands — the original operator goal; if the walk collapses back to the paddle under RL pressure, the BC zip remains the artifact and the lever is lr/KL, not architecture.

**gate**: PASS: 10M forensics — walk stays cheat-free (det gait_valid >=5/6, det med prog_ratio >=0.85, no parked-leg fingerprint) AND stance lifts vs the BC parent (rise det >=3/6 with >=1 crouch, lower det >=3/6, hold det 6/6) AND no canary walk regression fired. FAIL: walk fingerprint collapses to paddle or canaries auto-stop -> keep ppo_goal_cw_gru_bc.zip as artifact, next lever is smaller lr / tighter target-kl (NOT more steps, NOT from-scratch variants).

