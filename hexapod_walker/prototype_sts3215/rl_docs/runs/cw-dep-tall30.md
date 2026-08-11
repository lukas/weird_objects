# cw-dep-tall30

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-11T20:27:39+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-dep-tip1

**wandb_id**: ipmzx2kw

**hypothesis**: TALL DEPLOYABLE WALKER, rung 1 of the height-ref ladder (operator directive). The dep walker rides ~50mm below plant; dep-hgt1 proved a one-shot income gate cannot move it, but the lowgait line proved 10-20mm height-REFERENCE rungs train warm in single runs with 4-7mm tracking - height as a commanded task, not a gate. This rung: warm from cw-dep-tip1 (best hardware walker, full dep contract + tipped starts retained), walk_height_off_mm=-30 (~+21mm taller than the natural sag). Also carries the audit-derived structural stance-slip charge (k_drag_stance=8000/6mm/0.25mm, k_drag_loaded=0): measured today on a warm walker to be absorbed without parking and with slight slip improvement - at a taller stance with more swing clearance it has its best chance of pricing real stepping. If this rung tracks, next rungs are -15 then 0 (full plant height, physically proven walkable by the scripted hardware gait).

**gate**: Rung gate (lowgait standard): end-height err <=8mm at the -30 ref, det+sto gait_valid with vel err within 20% of parent tip1, slip/m <= parent band (no worse), zero park episodes, no falls; video visibly taller than parent. FAIL if height err >8mm or walk retention breaks - then retry without the drag charge to isolate.

