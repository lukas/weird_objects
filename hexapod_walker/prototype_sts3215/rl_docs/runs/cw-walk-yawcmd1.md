# cw-walk-yawcmd1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-10T03:15:55+00:00

**pod**: hexapod-mjx-train-2

**steps**: 12000000

**parent**: cw-walk-joyjit-dr05-c1

**hypothesis**: Yaw-rate command channel (code c086a22, probe clean): kernel k_walk_yaw=1.0 incl. wz_ref=0 heading-hold income trains commanded turning/arcs/turn-in-place on the jittered driving package (warm start joyjit-dr05-c1, obs 72->73 via pad transplant) without eroding linear driving. If false: parked-yaw free income dominates (yaw_err flat ~|wz_ref|) -> escalate to yaw income gating per WISHLIST item 3 risk note.

**gate**: own-cfg eval: commanded-turn segments |wz_err| med <= 0.10 rad/s AND wz_ref=0 segments |wz| med <= 0.05 rad/s (heading hold) AND JOYSTICK GATE retained (0 falls incl. flips) AND forward det med within parent band, slip <= 1.25

**refused_reason**: hexapod-mjx-train-2 already runs cw-walk-yawcmd1-rr1 — GPU pods host exactly one run; pick a free GPU pod.

