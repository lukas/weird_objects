# cw-walk-placementnoise6-payload-rr1-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T04:09:59+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: cw-walk-placementnoise6-r3

**hypothesis**: Compose: payload (1.0-1.5x) onto placementnoise6 (hand-placement-slop 6deg). 2nd requeue -- first attempt died with no ledger trace under the fleet's severe concurrent-drain launch-collision storm this window, 0 compute lost.

**gate**: Own-cfg (dr.placement_noise_deg=6.0 + dr.mass_scale=1.0,1.5) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd>=1.2m; DR0 flat retention det 6/6 gv, slip/m<=1.24, prog>=0.90; frames watched det

