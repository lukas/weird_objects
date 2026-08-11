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

**verdict**: MIXED (walk-retention PASS, stance-lift FAIL; operator cycle): 10M forensics gate+own-DR0.5 — walk gait_valid 6/6 in ALL FOUR passes (det+sto x DR0+DR0.5), zero sacrificed legs, prog med 0.76-0.85, speed on band; THE PADDLE NEVER RETURNED through 10M RL steps, walk economy softened (slip/m 2.1-2.6 vs BC parent 1.5). lower LIFTED 4-6/6 (parent det 2/6). rise 0/6 unchanged (only 60 BC demo episodes — never cloned, RL could not invent it); hold REGRESSED 0/6 (parent 6/6) — RL traded hold for locomotion polish. Root cause is distillation data poverty on stance, not architecture. Next lever (operator continuing): re-distill stance-heavy + DAgger rounds on distill_gru.py, harness-eval BC before any further RL. Artifacts: train-1 logs/ckpt_eval/cw_arch_gru_bc_ft1_{gate,owncfg}; ckpt md5 a4445c6b; BC parent ppo_goal_cw_gru_bc.zip md5 864c02fb stays the hold+walk artifact.

