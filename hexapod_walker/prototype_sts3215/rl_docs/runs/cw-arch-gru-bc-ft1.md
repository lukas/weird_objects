# cw-arch-gru-bc-ft1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-11T19:51:28+00:00

**pod**: hexapod-mjx-train-1

**steps**: 10000000

**wandb_id**: iv0ohvk5

**hardware_ready**: False

**hypothesis**: gru-r1..r4c post-mortem: the from-scratch GRU rung was budget-confounded (hist16-r7, the MLP reference, needed 40M walk-only steps; the GRU runs got 2M mixed) — from-scratch exploration finds the paddle first at discovery budget, but nothing is wrong with GRU+PPO itself (test_gru_policy.py incl. memory-task learning, all pass). So skip exploration: BC-distill both champions into one GRU (walks + holds out of the box), then PPO fine-tune with the anti-cheat stack on the mixed diet to lift rise/lower and polish. If true, one recurrent policy walks AND sits/stands — the original operator goal; if the walk collapses back to the paddle under RL pressure, the BC zip remains the artifact and the lever is lr/KL, not architecture.

**gate**: PASS: 10M forensics — walk stays cheat-free (det gait_valid >=5/6, det med prog_ratio >=0.85, no parked-leg fingerprint) AND stance lifts vs the BC parent (rise det >=3/6 with >=1 crouch, lower det >=3/6, hold det 6/6) AND no canary walk regression fired. FAIL: walk fingerprint collapses to paddle or canaries auto-stop -> keep ppo_goal_cw_gru_bc.zip as artifact, next lever is smaller lr / tighter target-kl (NOT more steps, NOT from-scratch variants).

**verdict**: FAIL vs stated gate: walk stays honest (gait_valid 6/6, prog_ratio median ~0.85, balanced duty, no exploit) but the 10M walk-heavy RL fine-tune erodes the BC-distilled stance skills — rise 0/6 det (all three start kinds, honest sprawl/stall to 21-108mm short, no flag-leg/park signature) and hold 0/6 det (track_err 2.43 deg vs 1.5 required — a real but small stillness miss, posture/height otherwise fine). Lower still lifts (4/6 det). Keep ppo_goal_cw_gru_bc.zip (pre-finetune BC artifact) as the reference; per the pre-registered gate action next lever is smaller lr / tighter target-kl, not more steps or a from-scratch variant.

