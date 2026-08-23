# cw-amp-m3-turnpush1-style05-acq1-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-23T01:59:52+00:00

**pod**: hexapod-mjx-train-1

**steps**: 6000000

**parent**: cw-amp-m3-turnpush1-style05-acq1

**wandb_id**: wlu57rof

**hypothesis**: Plain English: turning while being shoved looked bad at the 2M quick-look, but the robot was still learning fast when we stopped — give it the same full budget the non-turning push walker needed. Acquisition continuation of turnpush1-style05-r2 (turn-capable substrate + 10-25N single shove), 6M more from its own ckpt, dose unchanged — the direct test of the 'just needed more steps' read vs the sequential-composition alternative named in STATUS. Prediction-if-true: det prog med recovers toward the pushsmoke1-style05 band (>=0.9 vs r2's 0.37), topples <=1/6 det + <=2/6 sto, and eval_yaw tip err stays within ~0.05 of the substrate's 0.15-0.16 (turn not sacrificed for push survival). Prediction-if-false: reward flattens while prog stays <0.6 or turn tracking collapses to the park fingerprint (0.28-0.33) — budget refuted for the joint skill; the named next is SEQUENTIAL composition (solidify turn+push force staging separately) per the M2->M3->M4 milestone ordering. Strongest alternative: prog recovers but turn erodes (the yawcmd0 income-audit erosion mechanism re-fires under push) — then the yaw_prog_overshoot_decay/yaw_prog_avg_s repricing keys are the follow-up, pending the yppeak arm's readout. (Infra retry: verbatim relaunch of a REFUSED stale-pod-code-marker stub via respec --now sync path; no spec change.)

**gate**: Acquisition (6M continuation, 8M total, DR-0). PASS = own-cfg gate (push on, yaw cfg on) det prog med >=0.9, topples <=1/6 det + <=2/6 sto, gait_valid >=5/6 det+sto, zero sacrificed, AND eval_yaw tip errs <=0.21 (within 0.05 of substrate 0.15-0.16); video shows turning under shove absorption. INFORMATIVE-budget-refuted = reward flat with prog <0.6 => sequential composition is the named lever. INFORMATIVE-turn-eroded = prog recovers but tip errs >0.21 / park fingerprint => turn-vs-push interference, repricing keys next. FAIL = collapse/statue/NaN.

**verdict**: TURN+PUSH COMPOSITION CLOSES WITH BUDGET: the 6M acquisition continuation (8M total) clears every pre-registered PASS branch. Evidence: DR-0 own-cfg gate (push+yaw both on) det prog med 1.113 (bar >=0.9, was 0.37 at 2M), 0/6 det + 2/6 sto terminations (bar <=1/6 det + <=2/6 sto, both genuine push topples on video's last frame not statues -- tilt_pitch/tilt_roll after 6-9 clean strides), gait_valid 6/6 det+sto, zero sacrificed legs. eval_yaw (ran manually, matching phase-obs cfg): tip-left/right err 0.1431/0.1152, both <=0.21 and within 0.05 of the pre-push turn substrate (0.1525/0.1614) -- turn tracking was NOT sacrificed for push survival (tip-right actually improved). Video (det_0, sto_2/0/4) shows clean six-leg alternating gait with visible heading rotation under push, genuine falls only on the two sto topples. Why: the 2M discovery read (r2, prog 0.37) was undertrained per the 08-21 ruling, not a broken combination -- same pattern as push-alone and fault-alone, which also needed a 3x acquisition step past their 2M discovery reads. What's next: turn+push is now acquisition-solid, so the M2->M3->M4 sequential-composition route (grafting fault onto THIS checkpoint, rather than a fresh 3-way stack) becomes runnable -- launching cw-amp-m4-turnpushfault1-style05 this cycle.

