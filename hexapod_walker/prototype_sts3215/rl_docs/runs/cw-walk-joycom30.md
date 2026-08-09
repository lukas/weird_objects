# cw-walk-joycom30

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T19:44:30+00:00

**pod**: hexapod-mjx-train-11

**steps**: 18000000

**parent**: cw-walk-joylat25

**wandb_id**: 16n9ww3z

**hypothesis**: Driving with an off-center payload: the operator will strap batteries/cameras on the chassis and joystick it around; comshift30 proved the WALK champion handles +-30mm CoM shift on straight-line walking, but abrupt flips + DR + latency with a shifted CoM is untested (lateral flips against the offset are the risky case). One variable off joylat25: add dr.com_offset_m=0.03 to its DR0.5+latency abrupt-resample package. Plain: drive it around while it carries something off-center. If-true: joystick gate @DR0.2 zero falls AND own-cfg (com offset on) gv 12/12, 0 term, prog med >=0.80 - payload-tolerant driving banked. If-false: offset + flip interaction falls or craters progress on offset-against-turn draws - payload driving needs a dedicated exposure curriculum or stays a demo constraint (strap loads centered). Strongest alternative (0-for-3 lesson): joylat25 already covers it free - parent-baseline eval of joylat25 under the same offset runs BEFORE verdict and decides exposure-effect vs pre-existing tolerance.

**gate**: JOYSTICK GATE eval_drive @DR0.2 own cfg 0 in-envelope falls; own-cfg (DR0.5 + latency + dr.com_offset_m=0.03) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog med >=0.80; parent joylat25 baseline under identical offset cfg evaled alongside; DR0 retention det 6/6 gv; frames watched det

