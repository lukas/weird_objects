# cw-walk-payload-dr05-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-09T20:40:17+00:00

**pod**: hexapod-mjx-train-5

**steps**: 20000000

**parent**: cw-walk-payload50

**wandb_id**: 55ioa2yt

**hardware_ready**: False

**hypothesis**: Ruling-7 seed twin of the payload-dr05 FAIL (c61): that run's own-DR panel was clean (12/12) but DR0 no-payload retention eroded (slip 1.38>1.24, prog 0.54 vs 0.95) — the only dr05 compose so far to charge nominal, while comshift-dr05 and deadband-dr05 (c62) retained cleanly. Decide whether the mass-axis retention charge is real or seed luck before writing off the payload compose. ONE variable vs cw-walk-payload-dr05: seed 0 -> 1, identical config/parent. Plain: rerun the failed recipe with different dice - same failure means the axis is the problem. Prediction-if-true (fail repeats): DR0 no-payload retention erodes again (slip >1.24 or prog <0.75) - mass-axis DR compose genuinely charges nominal walking; class note becomes axis-dependent, payload stays a nominal-sim skill. Prediction-if-false: retention clean (slip <=1.24, prog ~0.95) - c61 was seed variance, requeue judgment on the compose. Strongest alternative: intermediate erosion - needs a third seed only if it decides a plan fork. Parent: cw-walk-payload50.

**gate**: Own-cfg harness at --dr-scale 0.5 + dr.mass_scale=1.0,1.5, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd >=1.1m; plus DR0 no-payload retention det 6/6 gv, det slip/m med <=1.24 AND prog med >=0.75; frames watched det

**verdict**: Ruling-7 seed twin PASS, CONTRADICTS c61's payload-dr05 (seed0) FAIL. Own-cfg panel (DR0.5+mass1.0-1.5): gv 6/6 det, 0 term, det med fwd 1.22m>=1.1m gate, slip 1.39/prog 0.87. DR0 no-payload retention: gv 6/6 det, 0 term, slip med 1.23<=1.24 gate (seed0 was 1.38, FAIL), prog med 0.93>=0.75 (seed0 was 0.54) - clean pass, not the seed0 erosion pattern. Video: all six legs cycling normally in both panels, same paddling-but-functional gait as champion, no new pathology, no flag leg. Interpretation: c61's retention erosion was SEED VARIANCE, not a systematic mass-axis-DR-charges-nominal-walking tax - the payload-dr05 compose class is now a 1-1 split (seed0 FAIL / seed1 PASS), needs a 3rd seed to settle per ruling-7 before treating either as final.

