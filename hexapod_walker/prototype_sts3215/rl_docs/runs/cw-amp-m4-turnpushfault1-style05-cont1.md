# cw-amp-m4-turnpushfault1-style05-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-23T03:10:48+00:00

**pod**: hexapod-mjx-train-2

**steps**: 6000000

**parent**: cw-amp-m4-turnpushfault1-style05-r2

**wandb_id**: wvk1berk

**hypothesis**: Plain English: matched control for ypfix1-r3 -- same checkpoint, same 6M budget, NO reward changes -- so any turn-calibration difference between the two runs is attributable to the overshoot repricing keys, not to more training. Prior budget-alone evidence (M2 acq1-r2: erosion WORSENED 0.15->0.38 over 6M; turnfault1-acq1-r5: tip park unchanged over 6M) is from different substrates; this closes the loop on THIS one. Prediction: yaw_ratio stays ~1.7 or drifts higher (overshoot farm deepens, per the signed income probe on the shared parent: wz_mean +0.478/-0.518, ratio 1.65/1.79, yaw_prog 291/294 via the legacy 1.25x clip), tips stay >0.30. Includes explicit trailing --obs-pad-transplant=0 (respec-from-consumed-transplant bug class, same fix as ypfix1-r3).

**gate**: Control (6M more, 8M total). Read jointly with ypfix1-r3: corrected-bus eval_yaw tips + income-probe yaw_ratio. EXPECTED = ratio >=1.5, tips >0.30 (budget alone does not repair overshoot farming) => keys-vs-control delta is the pricing effect, cite in the repricing readout. SURPRISE = tips <=0.20 here too => pricing was not the binding constraint on this substrate, re-audit before more reward keys. Safety floor either way: own-cfg gate gait_valid >=10/12, topples <=2/6+2/6 (erosion below r2's own 12/12 + 2/12 = composition cost of continued training, flag it).

**verdict**: Result: matched no-keys 6M control lands exactly on its own pre-registered EXPECTED branch -- budget alone does not repair the turn overshoot-farm. Evidence: own-cfg DR-0 gate matches parent r2 on safety with zero erosion (gait_valid 12/12 det+sto, topples 2/6 det [tilt_roll,tilt_pitch] + 0/6 sto = 2/12, identical count to r2's own 2/12; det prog med rose 0.785->0.95, slip fell 4.76->4.32). Signed income probe (probe_walk_income --stack yawcmd0) reproduces r2's overshoot fingerprint essentially unchanged: wz_mean +0.478/-0.518, yaw_ratio 1.65/1.79, yaw_prog income 291/294 through the legacy 1.25x clip -- 6M more steps under the same pricing left the optimizer pinned at the identical clamp-saturated attractor (md5-different checkpoint, functionally indistinguishable probe output). Hand-run eval_yaw (same fast-servo cfg-set both runs) confirms: tip-left/right err 0.4062/0.4104 (r2 0.4248/0.4932), both far over the 0.30 SURPRISE-branch bar -- and OTHER turn axes got WORSE with more training: fwd-hold spurious yaw 0.157->0.264 rad/s, arc-left 0.288->0.345, arc-right-max 0.320->0.436. Video (det_1/det_4, the two tilt topples): 5-10s of genuine clean six-leg tripod cycling then a real end-frame flip, not a statue/drag, matching r2's own read. Why: this IS the run's named EXPECTED outcome (ratio>=1.5, tips>0.30 => budget alone does not fix; the keys-vs-control delta is the pricing effect) -- confirms the misalignment diagnosis independently of the keys-ON sibling; per the 08-21 ruling this is not a lineage FAIL (safety floor exact, reward still mildly rising 195.3->205.7 Q3->Q4), it is the control half of a paired experiment. What's next: ypfix1-r3 (keys ON, RUNNING under a concurrent cycle) supplies the other half -- do not re-run this control; if ypfix1-r3 also misses <=0.20 tips, the M2 yppeak precedent (overshoot-decay only halved erosion there, hold/forward income dominance was the residual) predicts a second lever will be needed regardless of this control's own outcome.

