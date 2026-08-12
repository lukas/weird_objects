# cw-arch-gru-dual-scratch1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-12T13:08:40+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-arch-gru-scratch-anchor1

**wandb_id**: 8gvwwxya

**hardware_ready**: False

**hypothesis**: Rerun of cw-arch-gru-scratch-anchor1 (from-scratch GRU + full anchor stack, 2M) with ONE variable: the mode-gated dual-core architecture (DualGruActorCriticPolicy, commit 2137c00) instead of a shared trunk. scratch-anchor1 built hold 6/6 + lower 6/6 + rise 1-3/6 from RANDOM WEIGHTS in 2M — anchors work from scratch — but det walk still parked leg 1 in all six gate episodes (gait_valid 0/6): the shared trunk let stance-anchor traffic corrupt walk. With walk on its own core, the walk-tick anchor (kept ON exactly as parent — from-scratch walk RL alone paddles, r1-r4c) trains an isolated core toward TripodGait stepping, and stance anchors cannot touch it. Displacement is NOT expected at 2M (the walk anchor teaches in-place stepping; that is the finetune arm dual1 job); the question is whether isolation removes the det leg-park and preserves scratch stance quality.

**gate**: 2M forensics det+sto @DR0 gate cfg vs the frozen scratch-anchor1 report: PASS if det walk has ZERO sacrificed/parked legs with gait_valid >=4/6 (parent: leg 1 parked 6/6, gv 0/6) AND stance at least matches parent (hold det 6/6, lower det 6/6, rise sto >=3/6) AND anchor loss <0.02. FAIL if det walk still parks a leg (isolation does not fix the from-scratch park -> the park is a walk-core-internal optimum, from-scratch line stays closed regardless of architecture) OR stance regresses vs parent (splitting the trunk starves stance capacity at 2M).

**verdict**: FAIL (informative) per own gate: dual-core architecture DECISIVELY fixes the from-scratch walk leg-park -- det walk gait_valid 6/6 with ZERO sacrificed legs (parent scratch-anchor1: 0/6, leg1 parked in all 6 eps), confirming isolated cores remove the shared-trunk interference. Hold det 6/6 and lower det 6/6 both match parent. Misses ONE clause: rise sto 2/6 vs required >=3/6 (parent's own rate) -- det rise unchanged at 1/6 (matches parent exactly), so this is a narrow stochastic-only miss (n=6), not a collapse. bc_anchor_loss converged ~0.009-0.010, under the 0.02 bar. New residual to flag: own-DR0.5 walk partially re-fragments (gait_valid 3/6 vs parent's 5/6, legs 0/2 occasionally sacrificed) -- not gate-breaking (gate is DR0-only by design) but a watch item for the dual1 finetune.

