# cw-dep-startvar1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: KILLED

**created**: 2026-08-10T05:17:18+00:00

**pod**: hexapod-mjx-train-11

**steps**: 18000000

**parent**: cw-dep-vref1-r1

**hypothesis**: Operator directive 08-10 + GPT handoff item 7: the deployment-contract champion can absorb imperfect starts (placement +-6deg, bad starts p=0.4, logical-zero FRAME drift +-3deg where reads AND commands share the drifted frame — the failure that dropped the robot 08-09) without eroding the walk. If-true: own-cfg gait valid + varied-start eval clean, DR0 pristine-start retention in vref1-r1's band. If-false: zero-drift frame mode specifically breaks velocity-free walking (policy was leaning on cmd-vs-read residuals) -> dig-in on obs leakage, not more steps. DO NOT LAUNCH until cw-dep-vref1-r1 has a verdict AND its checkpoint exists at the init path; if vref1-r1 FAILS, re-parent to the walk champion instead.

**gate**: Own-cfg (startvar DR) det+sto 6/6 @15s: gait_valid 12/12, 0 falls, slip/m med<=1.6; DR0 pristine-start retention det 6/6 gv, prog med >=0.85 of vref1-r1's band; PLUS varied-start panel: 12 eps sampled with placement6/bad-start/zero-drift-frame active, >=10/12 gait valid, 0 falls; frames watched det+sto

**verdict**: INFRA/PROCESS: launched prematurely by ops.sh drain (self-repairing drain has no semantic-hold awareness) before its own DO-NOT-LAUNCH gate was met -- cw-dep-vref1-r1 is finished in W&B but NOT YET VERDICTED, and the P0 order requires holding startvar1 until that verdict lands (re-parent to walk champion if vref1-r1 FAILS). Caught at ~65k/18M steps (first PPO iteration, no science lost) and killed. Left OUT of backlog on purpose -- do not requeue until a cycle verdicts cw-dep-vref1-r1; then relaunch startvar1 fresh (same spec) parented per that verdict.

