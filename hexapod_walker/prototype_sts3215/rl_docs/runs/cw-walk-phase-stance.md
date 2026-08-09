# cw-walk-phase-stance

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: KILLED_RELAUNCH

**created**: 2026-08-08T20:41:51+00:00

**pod**: hexapod-sweep-walk

**steps**: 4000000

**parent**: ppo_goal_cw_stance_dr10.zip (md5 da1d912a, copied to walk pod as init_stance_dr10.zip)

**wandb_id**: o6m4zig3

**hypothesis**: Basin-escape arm (operator 20:25Z): all walk refutations warm-started from the converged shuffle; a policy that never learned it (stance champion, stands/rises/lowers at DR 1.0) plus the phase tripod reward from step 0 can find stepping. Fresh init already refuted round 1 (converged to the same skate), so stance init is the escape vehicle. If-true: stance-init steps (six-leg swing counts, gait-valid episodes) while dr04b arm shuffles -> the basin is the story; behavior-class changes get fresh-basin inits. If-false-both-shuffle: phase reward refuted independent of basin. If-both-step: keep the better gait. DR 0.2 start per operator addendum 20:35Z (skill-first; anneal 0.2->0.4->1.0 after stepping exists).

**gate**: sto walk >=4/6 gait-valid @ vel_err <=0.035 on 0.02-0.06 @ DR 0.2 AND video shows all six feet cycling contact/swing AND sto rise >=4/6 retained (canaries: all four groups protected from stance parent)

**verdict**: KILLED @~22.74M cum (~2.0M/4M) cycle 11c: operator best-practices audit (02ea8cc, binding) landed minutes after launch - basin-escape arms must use log_std_init 0.0 (std 1.0) + ent_coef 0.005-0.01 + target_kl 0.02; this run had std 0.37 / ent 1e-3 / no target_kl, i.e. exactly the under-exploration taint the audit calls out for from-scratch refutations. RESTARTED from the ORIGINAL stance init (not continued: the trained steps carry the taint) as cw-walk-phase-stance2 with audited settings. Not a scientific verdict.

