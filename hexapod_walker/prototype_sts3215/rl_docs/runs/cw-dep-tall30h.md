# cw-dep-tall30h

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-11T20:36:34+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-dep-tip1

**wandb_id**: 89pzr1vs

**hardware_ready**: False

**hypothesis**: TALL DEPLOYABLE WALKER rung 1, ISOLATION arm. cw-dep-tall30 (height ref -30 + k_drag_stance=8000) moved the body up (eval walk height_err_end 15.2mm vs parents 20-40mm off this ref) but missed the 8mm gate and halved walk speed to 0.0295 m/s while paying reward_drag_stance=-6.9/tick - the charge, not the ref, is the suspected gait suppressor. This arm: identical warm respec of cw-dep-tip1 with walk_height_off_mm=-30 as the ONLY change (parent drag stack k_drag_loaded=10 retained, no stance charge). Lowgait precedent: single-variable height-ref rungs track within 4-7mm in one warm run.

**gate**: PASS: eval walk height_err_end <=8mm at -30 ref AND walk speed >=0.045 m/s (>=75% of the 0.05-0.06 command band, tip1 retention) AND survived_frac 1 and no park. If PASS: next rung -15 then 0. If height tracks but speed still drops: the height rung itself costs speed - evaluate tradeoff with operator. If height err still >8mm: dep stack cannot track height refs the way the privileged lowgait line does - stop the ladder and report.

**verdict**: BACKFILL of the documented 08-11 read: charge-isolation twin of cw-dep-tall30 (no k_drag_stance) — identical speed/slip/height, proving the structural drag charge is FREE on a warm walker (GAIT.md ladder table). Same family closure as tall30: pricing ladder closed, wall broken by BC-INIT (cw-dep-bcgait1).

