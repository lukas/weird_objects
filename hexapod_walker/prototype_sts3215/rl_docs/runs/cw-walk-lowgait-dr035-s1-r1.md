# cw-walk-lowgait-dr035-s1-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-09T22:53:32+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-lowgait-dr035-s1

**hardware_ready**: False

**hypothesis**: Infra retry of cw-walk-lowgait-dr035-s1 (gotcha 13b: launch-collision EOFError under concurrent-cycle drain storm, 0 steps, no science result). IDENTICAL config, same seed 1. If-true: trains normally to completion this time and delivers the ruling-7 seed-twin evidence the original spec asked for. If-false: fails again -> escalate as a real infra issue, not gotcha 13b.

**gate**: Own-cfg DR0.35 15s at -50mm 6+6: gv 12/12, 0 term, mean end-height err <=10mm, slip/m med <=1.6; DR0 det retention gv 6/6, height err <=8mm, slip/m <=1.15; frames watched det

**verdict**: LAUNCH FAILURE again, not a science result: worker EOFError at env reset (gotcha 13b, launch-collision amid concurrent-cycle drain storm — status showed 4 other runs land in the same window), 0 steps, W&B run si8h0hjw crashed at init. Requeuing as -r2 for the self-repairing drain once contention clears.

**failed_reason**: run never appeared as 'running' in W&B within 240s

