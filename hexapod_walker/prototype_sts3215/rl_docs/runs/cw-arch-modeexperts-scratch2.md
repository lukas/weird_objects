# cw-arch-modeexperts-scratch2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-15T06:18:41+00:00

**pod**: hexapod-mjx-train-2

**steps**: 40000000

**parent**: cw-arch-modeexperts-scratch1-r1

**wandb_id**: 1t6rmexz

**hypothesis**: Give the four-specialist from-scratch brain its real training budget with the skill diet the operator actually ordered: the 2M canary proved the mechanism healthy but measured that sequence episodes overfeed HOLD (25% of real practice ticks vs 10% ordered) and starve WALKING (23% vs 35%), because holds come before walks in the sequence grammar and early falls cut walks short. This 40M stage continues the same random-init lineage (no imitation anywhere) with a corrected diet -- sequence episodes 50%->20%, single-mode mix re-solved from MEASURED sequence tick fractions -- so stand-up, walking and sit-down each get ~30% of real practice ticks (~12M each this stage). If the isolated-experts architecture can acquire the skills, per-mode eval scores move within this stage; a skill that stays flat under full measured exposure indicts its reward/start curriculum, not capacity or interference.

**gate**: ACQUISITION stage (pre-registered, MODE_EXPERTS_DIRECTIVE.md SCRATCH2): (a) completes 40M with no NaN/crash -- silent death gets ONE retry from the latest periodic checkpoint (save-every 1M) on a clean pod, infra not science; (b) EXPOSURE clause: realized experts/tick_frac_* within ±0.05 of rise/loco/lower .30 each and hold ≤.15 at 10M and at end -- a drift re-solves the scratch3 mix from measured f_seq, never a kill; (c) per-expert learning signal visible (independent stds + per-mode background C-env eval trends; aggregate return is NOT evidence). NO early stop for poor skill at any milestone -- stop only for NaN/crash, a proven exploit dominating a milestone video, or clause (b) instrumentation dying. Skill success is judged only at fork/final via bulk cohorts (fb_20260815T033634_7d750e); 40M total is NOT the requested budget -- scratch3 (pre-registered below) tops REAL active ticks to ~20M/skill.

