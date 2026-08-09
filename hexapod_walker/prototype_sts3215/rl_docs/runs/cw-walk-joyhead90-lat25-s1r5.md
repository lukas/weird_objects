# cw-walk-joyhead90-lat25-s1r5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-09T23:11:09+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-joyhead90-lat25-s1r4

**hypothesis**: 5th consecutive infra retry of the joyhead90-lat25 seed-1 twin (all prior attempts died to the host-wide EOFError launch-collision storm, 0 steps each). Same spec unchanged; queued to backlog.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 --heading-max-deg 90 own cfg - ZERO in-envelope falls, left/right dist >=0.15m; own-cfg DR0.5+latency harness 6+6: gv 12/12, 0 term, prog_ratio med >=0.80; frames watched det

**verdict**: INFRA (not a science result): EOFError at env-reset, 0 steps -- 6th CONSECUTIVE failure of this exact spec (seed-1 twin of joyhead90-lat25), while sibling infra-retries this same window (imupos15-r3, placementnoise6-r3) succeeded on their first or second retry. Checkpoint md5 verified clean on both controller and pod. Six-in-a-row for one spec is beyond the generic collision-storm noise band this cycle documented elsewhere -- flagging for a closer look at rl_move/sim/mjx_sharded_vec_env.py reset choreography rather than another blind respec. NOT requeuing a 7th time from triage; leaving for a dig-in look.

