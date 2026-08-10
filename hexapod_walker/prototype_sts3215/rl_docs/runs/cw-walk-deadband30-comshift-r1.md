# cw-walk-deadband30-comshift-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: KILLED

**created**: 2026-08-10T05:18:44+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-deadband30

**hypothesis**: Retry of cw-walk-deadband30-comshift (self-repair PARKED it after 3 launch attempts, all failing on a transient /tmp/proto_sync.tgz write-race on the target pod during a concurrent sync -- confirmed both pods can write fine now, not a real permission fault). Same spec unchanged: off-center CoM payload shift (0.03m) x servo deadband hardening (1.0-3.0x) -- untried pairing off the isolated walk-only deadband30 package.

**gate**: Own-cfg (dr.deadband_scale=1.0,3.0 + dr.com_offset_m=0.03) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det prog med>=0.85; DR0 flat-nominal retention det 6/6 gv, slip/m<=1.24; frames watched det

**verdict**: Killed as a duplicate: two separate self-repair launches (this -r1 retry and the original cw-walk-deadband30-comshift) both landed as live training pods for the identical spec after the earlier sync-race PARKing. Kept the original (equal progress, arbitrary tie-break), killed this one to free the GPU pod; 0 science lost.

