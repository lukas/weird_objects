# cw-walk-placementnoise6-payload-rr1-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T04:09:59+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: cw-walk-placementnoise6-r3

**wandb_id**: hisp0uc4

**hardware_ready**: False

**hypothesis**: Compose: payload (1.0-1.5x) onto placementnoise6 (hand-placement-slop 6deg). 2nd requeue -- first attempt died with no ledger trace under the fleet's severe concurrent-drain launch-collision storm this window, 0 compute lost.

**gate**: Own-cfg (dr.placement_noise_deg=6.0 + dr.mass_scale=1.0,1.5) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd>=1.2m; DR0 flat retention det 6/6 gv, slip/m<=1.24, prog>=0.90; frames watched det

**verdict**: PASS -- payload (1.0-1.5x mass) composes cleanly onto placementnoise6 (6deg hand-placement-slop). Own-cfg (placement6+payload) det+sto 12/12 gait_valid, 0 term, 0 falls, det med fwd 1.33m (>=1.2 gate); 2/6 det draws crater to a shuffle (fwd 0.5-0.58m, slip 3.2-3.5) -- the same known hand-placement-slop shuffle tail the parent placementnoise6-r3 showed (3/6 there), actually milder here (2/6) with payload added. DR0 flat (no noise, no payload) retention is clean: det med slip/m 1.21 (<=1.24 gate, just inside), prog 0.91 (>=0.90); sto clean 1.08/0.97, gv 12/12, 0 term across both passes -- no erosion from the compose. Frames watched on the own-cfg det pass: level six-leg cycling, no flag leg, same known paddle/foot-slide gait as the champion -- not a new defect, not hardware-ready.

