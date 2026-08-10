# cw-walk-placementnoise6-comshift

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T05:58:06+00:00

**pod**: hexapod-mjx-train-5

**steps**: 20000000

**parent**: cw-walk-placementnoise6-r3

**wandb_id**: xnzk1aqz

**hypothesis**: NEW compose, untried pairing: off-center CoM payload shift (0.03m) x hand-placement-slop hardening (placementnoise6, 6deg, PASS). CoM offset composes for free onto the plain champion, groundtilt5, and multiple driving packages; hand-placement-slop specifically has not been tried with an off-axis CoM load. If-true: own-cfg (placementnoise6+comshift) det+sto 6/6 gv, 0 term, det prog med>=0.85 (matching placementnoise6's own band); DR0 flat-no-slop-no-offset retention clean. If-false: the off-axis load interacts with placement-slop errors to crater progress or cost falls that flat-ground comshift composes did not show. (Requeue: 1st queue attempt vanished from backlog with zero ledger trace, matches the documented lost-update symptom under this cycle's heavy concurrent-drain load, 0 compute lost.)

**gate**: Own-cfg (dr.placement_noise_deg=6.0 + dr.com_offset_m=0.03) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det prog med>=0.85; DR0 flat-no-slop-no-offset retention det 6/6 gv, slip/m<=1.24, prog>=0.90; frames watched det

