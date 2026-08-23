# cw-amp-m4-turnpushfault1-style05-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-23T03:10:48+00:00

**pod**: hexapod-mjx-train-2

**steps**: 6000000

**parent**: cw-amp-m4-turnpushfault1-style05-r2

**hypothesis**: Plain English: matched control for ypfix1-r3 -- same checkpoint, same 6M budget, NO reward changes -- so any turn-calibration difference between the two runs is attributable to the overshoot repricing keys, not to more training. Prior budget-alone evidence (M2 acq1-r2: erosion WORSENED 0.15->0.38 over 6M; turnfault1-acq1-r5: tip park unchanged over 6M) is from different substrates; this closes the loop on THIS one. Prediction: yaw_ratio stays ~1.7 or drifts higher (overshoot farm deepens, per the signed income probe on the shared parent: wz_mean +0.478/-0.518, ratio 1.65/1.79, yaw_prog 291/294 via the legacy 1.25x clip), tips stay >0.30. Includes explicit trailing --obs-pad-transplant=0 (respec-from-consumed-transplant bug class, same fix as ypfix1-r3).

**gate**: Control (6M more, 8M total). Read jointly with ypfix1-r3: corrected-bus eval_yaw tips + income-probe yaw_ratio. EXPECTED = ratio >=1.5, tips >0.30 (budget alone does not repair overshoot farming) => keys-vs-control delta is the pricing effect, cite in the repricing readout. SURPRISE = tips <=0.20 here too => pricing was not the binding constraint on this substrate, re-audit before more reward keys. Safety floor either way: own-cfg gate gait_valid >=10/12, topples <=2/6+2/6 (erosion below r2's own 12/12 + 2/12 = composition cost of continued training, flag it).

