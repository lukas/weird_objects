# cw-arch-hist16-dep1-s1r1-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PARTIAL

**created**: 2026-08-23T18:12:34+00:00

**pod**: hexapod-mjx-train-9

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-s1r1

**wandb_id**: 3v67spen

**hypothesis**: Plain English: operator-requested replication — does the impressive hist16-dep1 -> c1 continuation result reproduce on the seed twin? cw-arch-hist16-dep1-c1 (+40M continuation, seed 0) closed the dep-contract walker's efficiency gap and PASSed; the from-scratch recipe already replicated on seed 1 (cw-arch-hist16-dep1-s1r1, PASS, det slip 1.28-1.35). This run is the missing closest replication of the continuation step: continue s1r1 for +40M with c1's exact config (joint_walk, DR0.5, hist16, deployment contract walk_obs_body_vel=2, same walk reward/goal cfg), changing only parent/init/name and keeping s1r1's seed 1. If-true: slip/economy improves or holds toward c1's levels (det ~<=1.25, sto ~<=1.37) with 6/6 gait_valid, 0 term, joystick gate 0 falls — the c1 result is a reproducible recipe, not seed-0 luck. If-false: seed-1 continuation stalls or degrades — the c1 gain is seed-specific or noise, and the lineage claim needs an audit before any further continuation. Per operator note 20260823T180148Z: if reward rises while eval/visual gait worsens, stop and audit reward/eval disagreement instead of running more seeds.

**gate**: vs parent s1r1 and vs c1: own-cfg DR0.5 det+sto 6/6 gait_valid, 0 terminations; DR0 gate det+sto 6/6 gv, 0 term; joystick gate (eval_drive) 0 falls; video visually ordinary six-leg gait, no flag leg; slip/m improves or holds from s1r1's det 1.28-1.35 toward c1 levels (target det <=~1.25, sto <=~1.37)

**verdict**: Reproduction of the dep1->c1 slip-improvement recipe on seed-1 twin is MIXED, not clean. Safety/joystick fully replicates: 0 falls across DR0 gate (12 eps), own-DR0.5 gate (12 eps), and eval_drive joystick panel (fwd/back/left/right/diag/stop-go/flip-stress, all 0 falls, trk_err 0.03-0.07 m/s) -- gait_valid 6/6 every mode, video ordinary six-leg cycling, no flag leg. But the targeted slip/economy gain does NOT reproduce under domain randomization, the harder axis: DR0 gate improves in the right direction but undershoots target (det slip 1.35->1.30 vs target <=1.25; sto 1.43->1.36, meets <=1.37) while own-DR0.5 sto REGRESSES 30% (1.39->1.81, target <=1.37, violates the gate's own 'improves or holds') and own-DR0.5 det is flat (1.28->1.29, misses 1.25). Reward rose monotonically the whole +40M run (782->894 quarters), so this is not undertraining -- it is seed variance in the c1 recipe's robustness gain, matching the run's own pre-registered if-false branch ('the c1 gain is seed-specific or noise, and the lineage claim needs an audit before any further continuation'). Evidence: logs/ckpt_eval/cw_arch_hist16_dep1_s1r1_c1_{gate,owncfg,drive.json}. Out-of-scope operator lineage (track=arch, not in tracks.json) -- honest triage recorded, no agent follow-up launched; audit/next-seed decision is the operator's/that lineage owner's call.

