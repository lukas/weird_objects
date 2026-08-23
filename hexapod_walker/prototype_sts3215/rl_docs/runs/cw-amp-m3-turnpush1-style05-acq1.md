# cw-amp-m3-turnpush1-style05-acq1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-23T01:53:49+00:00

**pod**: hexapod-mjx-train-0

**steps**: 6000000

**parent**: cw-amp-m3-turnpush1-style05-r2

**hypothesis**: Plain English: turning while being shoved looked bad at the 2M quick-look, but the robot was still learning fast when we stopped — give it the same full budget the non-turning push walker needed. Acquisition continuation of turnpush1-style05-r2 (turn-capable substrate + 10-25N single shove), 6M more from its own ckpt, dose unchanged — the direct test of the 'just needed more steps' read vs the sequential-composition alternative named in STATUS. Prediction-if-true: det prog med recovers toward the pushsmoke1-style05 band (>=0.9 vs r2's 0.37), topples <=1/6 det + <=2/6 sto, and eval_yaw tip err stays within ~0.05 of the substrate's 0.15-0.16 (turn not sacrificed for push survival). Prediction-if-false: reward flattens while prog stays <0.6 or turn tracking collapses to the park fingerprint (0.28-0.33) — budget refuted for the joint skill; the named next is SEQUENTIAL composition (solidify turn+push force staging separately) per the M2->M3->M4 milestone ordering. Strongest alternative: prog recovers but turn erodes (the yawcmd0 income-audit erosion mechanism re-fires under push) — then the yaw_prog_overshoot_decay/yaw_prog_avg_s repricing keys are the follow-up, pending the yppeak arm's readout.

**gate**: Acquisition (6M continuation, 8M total, DR-0). PASS = own-cfg gate (push on, yaw cfg on) det prog med >=0.9, topples <=1/6 det + <=2/6 sto, gait_valid >=5/6 det+sto, zero sacrificed, AND eval_yaw tip errs <=0.21 (within 0.05 of substrate 0.15-0.16); video shows turning under shove absorption. INFORMATIVE-budget-refuted = reward flat with prog <0.6 => sequential composition is the named lever. INFORMATIVE-turn-eroded = prog recovers but tip errs >0.21 / park fingerprint => turn-vs-push interference, repricing keys next. FAIL = collapse/statue/NaN.

**refused_reason**: hexapod-mjx-train-0 code marker 331a459014c31c4796e32cb27ff66067468c8749 != local HEAD 8bb6ade3192610acc1e9152bcdd11575f16a75d4. Sync first: snapshot.sh --sync hexapod-mjx-train-0 (and snapshot/commit before that if the tree is dirty).

