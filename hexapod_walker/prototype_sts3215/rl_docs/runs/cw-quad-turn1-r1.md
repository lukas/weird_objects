# cw-quad-turn1-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T07:02:57+00:00

**pod**: hexapod-mjx-train-3

**steps**: 10000000

**parent**: cw-quad-hold2

**wandb_id**: choi0t15

**hypothesis**: Retry of cw-quad-turn1 (died at init: obs 72 vs 73 mismatch, I omitted --obs-pad-transplant 1 -- fixed here, 0 science lost). Quad-hold (30% mix) x yaw-rate turning command -- composes two independently-landed mechanisms (quad goal-mix c086a22, yaw-rate channel c086a22, obs 72->73 via pad transplant same as yawcmd1) that have never been trained together; tests WISHLIST 15's 'quad turn' rung, which was marked [CODE] on the assumption of needing new machinery -- both flags already exist so this may be launchable without new code. If-true: quad hold survives while walk segments turn (heading-hold+commanded-turn |wz_err| within the yawcmd gate) AND slip/m<=1.25. If-false: the combined obs/reward pressure breaks one skill or the other -- names the specific interaction for a future [CODE] fix rather than guessing.

**gate**: own-cfg (quad=0.3/walk=0.6/hold=0.1 + yaw_cmd) det+sto gv 6/6, 0 term; quad survived_frac>=0.9; walk-mode commanded-turn |wz_err| med<=0.10 rad/s, zero-seg |wz| med<=0.05 rad/s, slip/m<=1.25; frames watched det on walk and quad segments

