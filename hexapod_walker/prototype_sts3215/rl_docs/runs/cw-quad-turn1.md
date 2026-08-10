# cw-quad-turn1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T06:51:06+00:00

**pod**: hexapod-mjx-train-10

**steps**: 10000000

**parent**: cw-quad-hold2

**hypothesis**: Quad-hold (30% mix) x yaw-rate turning command -- composes two independently-landed mechanisms (quad goal-mix c086a22, yaw-rate channel c086a22) that have never been trained together; tests WISHLIST 15's 'quad turn' rung, which was marked [CODE] on the assumption of needing new machinery -- both flags already exist and obs width is unaffected by quad (one-hot, no width change) while yaw appends via pad-transplant, so this may be launchable without new code. If-true: quad hold survives while walk segments turn (heading-hold+commanded-turn |wz_err| within the yawcmd gate) AND slip/m<=1.25. If-false: the combined obs/reward pressure breaks one skill or the other (four-leg hold destabilizes under yaw torque, or turning doesn't track while quad is active) -- names the specific interaction for a future [CODE] fix rather than guessing.

**gate**: own-cfg (quad=0.3/walk=0.6/hold=0.1 + yaw_cmd) det+sto gv 6/6, 0 term; quad survived_frac>=0.9; walk-mode commanded-turn |wz_err| med<=0.10 rad/s, zero-seg |wz| med<=0.05 rad/s, slip/m<=1.25; frames watched det on walk and quad segments

**failed_reason**: run never appeared as 'running' in W&B within 240s

