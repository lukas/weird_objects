# cw-mt-widen1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-12T21:58:29+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-mt-b1

**wandb_id**: 2ifqhivx

**hardware_ready**: False

**hypothesis**: Take the specialist robot that already walks cleanly (cw-mt-a2, 20M) and teach it the extra commands — turn a little, stop and stand still — instead of training everything at once from scratch; this arm tests whether starting from a walking checkpoint avoids the interference that broke the from-scratch generalists. Recipe is cw-mt-b1 unchanged (vx 0-0.06, yaw +-0.15 on 20% of segments, 40% stop segments); the ONLY variable vs b1 is the init (a2 warm-start vs fresh). This is the 'staged widening from a walking checkpoint' lever pre-registered when wave 1 closed (multitask/STATUS.md Next). Prediction-if-true: the gait survives widening (gait_valid majority, prog near a2's) and stop/yaw segments show sign-correct acquisition within 2M. Prediction-if-false (forgetting): the walking prior collapses back into the crouch/paddle under the widened command distribution — per MULTITASK.md that labels FORGETTING, and the next lever is a gradual command-width curriculum, not more steps. Strongest alternative: gait retained but yaw never responds (acquisition failure, mirrors b2) — points at representation, not curriculum.

**gate**: At 2M: PASS = det gait_valid >=4/6 (gate DR0) AND det prog med >=0.6 (retains half of a2's 1.23) AND zero-command segments hold still AND eval_yaw probe turns sign-correct BOTH directions -> harden at 20M as wave-2 mainline. FAIL(forgetting) if gait_valid <=1/6 or prog med <0.3 (crouch/paddle relapse) -> next lever is staged/gradual command-width curriculum, no straight retry. FAIL(acquisition) if gait retained but yaw sign-response absent both directions -> representation lever, judged against b2's 0.137 |wz_err| / 9-fall baseline. Report drag/slip_per_m and roll_tail vs a2 in the verdict regardless.

**verdict**: FAIL(acquisition) per pre-registered branch — but the staged-widening half is CONFIRMED: the a2 walking prior FULLY survives command widening (gate DR0 det gait_valid 6/6, prog med 1.57 vs a2's 1.23; own-DR0.2 6/6/1.81; zero terms, no sacrificed legs, duty_min 0.23-0.44 — b2's leg-3 near-sacrifice absent; roll_tail 0.5-0.9deg flat-to-better vs a2; one quality cost: slip_per_m 1.92-2.19 vs a2's 1.38-1.50). Neither NEW command acquired at 2M: signed probe shows command-INVARIANT velocity (stop-hold speed_med 0.058 m/s == fwd-hold's 0.058 — marches through zero-commands) and yaw sign-response absent both directions (tip-left wz +0.103 vs tip-right +0.086, wrong sign right; eval_yaw matched to b2: turn |wz_err| med 0.155 vs b2 0.137, hold |wz| 0.133 vs 0.050, but 0 falls vs b2's 9). Budget confound on the acquisition read (no mt arm ever acquired anything at 2M; b2 needed 20M from scratch for partial yaw) — discriminator cw-mt-widen2 queued at the b2-matched 20M before declaring the representation lever.

