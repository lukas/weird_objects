# cw-walk-placementnoise6-payload-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-10T03:49:17+00:00

**pod**: hexapod-mjx-train-7

**steps**: 20000000

**parent**: cw-walk-placementnoise6-r3

**hypothesis**: Compose: payload (1.0-1.5x) onto placementnoise6 (hand-placement-slop 6deg). 2nd requeue -- first attempt died with no ledger trace under the fleet's severe concurrent-drain launch-collision storm this window, 0 compute lost.

**gate**: Own-cfg (dr.placement_noise_deg=6.0 + dr.mass_scale=1.0,1.5) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd>=1.2m; DR0 flat retention det 6/6 gv, slip/m<=1.24, prog>=0.90; frames watched det

**refused_reason**: hexapod-mjx-train-7 code marker bc01372eafd463f4f378c3e326f583f0e3c362e5-dirty != local HEAD bc01372eafd463f4f378c3e326f583f0e3c362e5. Sync first: snapshot.sh --sync hexapod-mjx-train-7 (and snapshot/commit before that if the tree is dirty).

