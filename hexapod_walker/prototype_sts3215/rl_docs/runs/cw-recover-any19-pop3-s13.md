# cw-recover-any19-pop3-s13

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INVALID_INTEGRATION_CANARY

**created**: 2026-08-17T23:56:41+00:00

**pod**: hexapod-mjx-train-3

**steps**: 40000000

**parent**: cw-recover-any18-pop3-s13

**wandb_id**: 79ef86ae

**hardware_ready**: False

**hypothesis**: Teach the robot to stand back up from any fallen position by racing three fresh identical-recipe seeds in lockstep and always adopting the first retention-clean winner at each curriculum bucket; this launch (member 2, seed 13, of population recover-any19-pop3, roster s11,s12,s13) replaces eventually-consistent display-name peer discovery (the mechanism behind FOUR distinct any16/17/18 sync bugs) with predeclared, immutable W&B run IDs (6907573e,1c67c001,79ef86ae) assigned before training. Exact unchanged training recipe otherwise. Absolutely NO --init-from, from scratch. Supersedes the INVALID any16/17/18 cohorts.

**gate**: Live integration gate (operator fb_20260817T234449_bcdcce, predeclared-id peer discovery) -- see cw-recover-any19-pop3-s11 for the full 7-point gate text (identical across the cohort).

**verdict**: INVALID_INTEGRATION_CANARY — same cohort story as any19-s11/s12 (root cause + evidence there and in q_20260817T2352Z addendum): this member (started LAST) saw all peers fine — zero 'not found' lines, consistent with the root cause (only runs created after a process's first W&B connect are invisible to it) — but the leader could never see s12/s13, so start_B00 was never released. Hung past its own 900s deadline blocked in-process (fail-closed timeout unenforceable); killed manually at ~00:17Z by the checkup cycle. ready_B00 valid, no training past 655,360. No behavioral conclusions.

