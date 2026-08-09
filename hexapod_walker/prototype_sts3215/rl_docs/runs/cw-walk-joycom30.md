# cw-walk-joycom30

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T19:44:30+00:00

**pod**: hexapod-mjx-train-11

**steps**: 18000000

**parent**: cw-walk-joylat25

**wandb_id**: 16n9ww3z

**hardware_ready**: no

**hypothesis**: Driving with an off-center payload: the operator will strap batteries/cameras on the chassis and joystick it around; comshift30 proved the WALK champion handles +-30mm CoM shift on straight-line walking, but abrupt flips + DR + latency with a shifted CoM is untested (lateral flips against the offset are the risky case). One variable off joylat25: add dr.com_offset_m=0.03 to its DR0.5+latency abrupt-resample package. Plain: drive it around while it carries something off-center. If-true: joystick gate @DR0.2 zero falls AND own-cfg (com offset on) gv 12/12, 0 term, prog med >=0.80 - payload-tolerant driving banked. If-false: offset + flip interaction falls or craters progress on offset-against-turn draws - payload driving needs a dedicated exposure curriculum or stays a demo constraint (strap loads centered). Strongest alternative (0-for-3 lesson): joylat25 already covers it free - parent-baseline eval of joylat25 under the same offset runs BEFORE verdict and decides exposure-effect vs pre-existing tolerance.

**gate**: JOYSTICK GATE eval_drive @DR0.2 own cfg 0 in-envelope falls; own-cfg (DR0.5 + latency + dr.com_offset_m=0.03) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog med >=0.80; parent joylat25 baseline under identical offset cfg evaled alongside; DR0 retention det 6/6 gv; frames watched det

**verdict**: NO-EFFECT (gate letter-PASS). Joystick gate PASS @DR0.2 (0 falls incl flip-stress); own-cfg DR0.5+latency+com30 gv 12/12, 0 term, det prog med 0.95/slip 1.50, frames clean (level, six legs cycling). But parent joylat25 under IDENTICAL offset cfg matches episode-for-episode (det 0.93/1.40, sto 0.97/1.61 vs 0.98/1.61) — all deltas inside eval noise. ±30mm CoM offset is already covered FREE by the joylat25 driving package; DR0 retention gv 6/6. joylat25 stays the driving candidate; no payload-driving curriculum needed — loads within ±30mm are covered.

