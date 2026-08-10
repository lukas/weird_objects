# cw-walk-yawcmd1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T06:42:10+00:00

**pod**: hexapod-mjx-train-7

**steps**: 12000000

**parent**: cw-walk-yawcmd1-rr1

**wandb_id**: ycemc5zk

**hypothesis**: Seed twin of cw-walk-yawcmd1-rr1 (P0/turning: yaw-rate command channel on the jittered driving package) -- yawcmd1-rr1 finished 12M/12M unverdicted (own-cfg gv 12/12 0 term, JOYSTICK GATE PASS 0 falls, forward slip 1.21/1.22<=1.25 gate -- but the yaw-tracking-specific metric (|wz_err| on commanded-turn segments vs |wz| on zero segments) needs custom analysis beyond the standard harness table, flagged DIG-IN). This seed twin gets a second data point ready in parallel rather than waiting serially, per the plan's 'idle pods during deliberation is a failure mode' rule.

**gate**: Own-cfg eval: commanded-turn segments |wz_err| med <= 0.10 rad/s AND wz_ref=0 segments |wz| med <= 0.05 rad/s (heading hold) AND JOYSTICK GATE retained (0 falls incl. flips) AND forward det med within parent band, slip <= 1.25 -- same gate as rr1, seed-robustness check

