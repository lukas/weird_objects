# cw-walk-lowgait-dr035-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-09T22:34:31+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-lowgait-dr035

**hardware_ready**: False

**hypothesis**: Seed twin of this cycle's cw-walk-lowgait-dr035 PASS (ruling-7: seed-confirm before banking a DR ceiling). Identical config: -50mm crouch warm-start, dr-scale 0.35, seed 1 instead of 0. If-true: seed 1 matches -- own-cfg DR0.35 gv 12/12, 0 term, mean height err <=10mm, slip/m med <=1.6, DR0 retention clean. If-false: seed 1 charges nominal retention or breaks under DR0.35 -- the 0.35 ceiling is seed-fragile, panel required before folding into consolidation. Parent: rl_move/sim/policies/ppo_goal_cw_walk_lowgait_dr035.zip.

**gate**: Own-cfg DR0.35 15s at -50mm 6+6: gv 12/12, 0 term, mean end-height err <=10mm, slip/m med <=1.6; DR0 det retention gv 6/6, height err <=8mm, slip/m <=1.15; frames watched det

**verdict**: LAUNCH FAILURE, not a science result: worker EOFError at env reset (gotcha 13b, launch-collision amid concurrent-cycle drain storm), 0 steps, W&B run 377epcui crashed at init. No training happened; no verdict on the hypothesis. Requeued to backlog as cw-walk-lowgait-dr035-s1-r1.

