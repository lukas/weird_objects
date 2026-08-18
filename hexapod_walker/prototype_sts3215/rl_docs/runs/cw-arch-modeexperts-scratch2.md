# cw-arch-modeexperts-scratch2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-15T06:18:41+00:00

**pod**: hexapod-mjx-train-2

**steps**: 40000000

**parent**: cw-arch-modeexperts-scratch1-r1

**wandb_id**: 1t6rmexz

**hardware_ready**: False

**hypothesis**: Give the four-specialist from-scratch brain its real training budget with the skill diet the operator actually ordered: the 2M canary proved the mechanism healthy but measured that sequence episodes overfeed HOLD (25% of real practice ticks vs 10% ordered) and starve WALKING (23% vs 35%), because holds come before walks in the sequence grammar and early falls cut walks short. This 40M stage continues the same random-init lineage (no imitation anywhere) with a corrected diet -- sequence episodes 50%->20%, single-mode mix re-solved from MEASURED sequence tick fractions -- so stand-up, walking and sit-down each get ~30% of real practice ticks (~12M each this stage). If the isolated-experts architecture can acquire the skills, per-mode eval scores move within this stage; a skill that stays flat under full measured exposure indicts its reward/start curriculum, not capacity or interference.

**gate**: ACQUISITION stage (pre-registered, MODE_EXPERTS_DIRECTIVE.md SCRATCH2): (a) completes 40M with no NaN/crash -- silent death gets ONE retry from the latest periodic checkpoint (save-every 1M) on a clean pod, infra not science; (b) EXPOSURE clause: realized experts/tick_frac_* within ±0.05 of rise/loco/lower .30 each and hold ≤.15 at 10M and at end -- a drift re-solves the scratch3 mix from measured f_seq, never a kill; (c) per-expert learning signal visible (independent stds + per-mode background C-env eval trends; aggregate return is NOT evidence). NO early stop for poor skill at any milestone -- stop only for NaN/crash, a proven exploit dominating a milestone video, or clause (b) instrumentation dying. Skill success is judged only at fork/final via bulk cohorts (fb_20260815T033634_7d750e); 40M total is NOT the requested budget -- scratch3 (pre-registered below) tops REAL active ticks to ~20M/skill.

**verdict**: ACQUISITION-STAGE PASS (pre-registered MODE_EXPERTS_DIRECTIVE.md SCRATCH2 gate) -- mechanism/exposure clauses met, no proven exploit, skill genuinely still immature -> continuing to scratch3. (a) finished clean 40.04M steps, no NaN/crash. (b) exposure drifted as pre-classified: at 10M rise .394 (miss-high), loco .283/lower .299 (in-band), hold .024 (ok); at end rise .312 (in-band), loco .236/lower .241 (miss-low by ~0.01-0.06), hold .211 (miss-high, 2x the .15 cap) -- driven by sequence episodes' hold segment absorbing more ticks as rise got reliably completed (more sequences reach hold), never a kill per the gate's own letter, re-solves the scratch3 mix below. (c) per-expert stds diverged independently (hold 1.84 > rise 1.51 > lower 1.33 > loco 0.74, all from a shared 0.39 init) and per-mode in-loop eval trends improved (hold survived_frac 0->1.0 by ~12M, walk/lower stayed high, rise mixed) -- real per-expert learning signal. Bulk harness read (DR0 gate + DR0.5 owncfg, det+sto, 6 eps/mode): 0/6 success EVERYWHERE, but NOT a dominating exploit -- walk has genuine 6/6 gait_valid six-leg cycling both DR passes (video-confirmed), 0 sacrificed legs, real forward travel (1.4-1.8m/30s matching the 0.05-0.06 m/s command) and prog_ratio ~1.0-1.1, just too much slip to clear the success bar (slip/m 1.8-2.4 vs a trained champion's ~1.0-1.3); rise curls upward genuinely (video-confirmed) but stalls short and over-currents in half the deterministic episodes (3/6 TERM both DR passes, worst_clear ~100-140mm); lower descends genuinely (video-confirmed) but never reaches the flat-plant target (worst_clear ~275-305mm, high foot-drag ~1.0-1.6m/ep, 0 terminations); hold stays upright and stable (video-confirmed, roll tail 0.4-0.8deg) but off the target height band (worst_clear ~130-215mm). Reads as real, uniform under-training across all 4 skills at this budget (~9-13M active ticks/skill so far), not a reward/eval bug and not a park/flag-leg/freeze cheat -- squarely the 'still learning' case the ACQUISITION gate explicitly disclaims judging. Per pre-registration: scratch3 launched same cycle, mix re-solved from measured cumulative deficits (see MODE_EXPERTS_DIRECTIVE.md), targeting >=20M cumulative active ticks/skill for rise/loco/lower.

