# cw-amp-m4-pushfault1-noamp-acq1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-23T02:30:23+00:00

**pod**: hexapod-mjx-train-2

**steps**: 6000000

**parent**: cw-amp-m4-pushfault1-noamp-r2

**hypothesis**: Plain English: the robot already handles a shove AND a broken joint in the same episode after just 2M steps of practice — this run gives it the full 6M practice budget that perfected the solo push skill, to see if the combined skill reaches the same solo-axis quality bars. Pre-registered continuation from pushfault1-noamp-r2's PASS branch (its gate: 'clears with reward still rising -> acquisition continuation matching pushacq1's 6M dose'; reward was 23/114/212/281 by quarter, steeply rising at cutoff). Warm from r2's checkpoint, byte-identical cfg, 6M more (8M total). Prediction-if-true: own-cfg gate reaches BOTH solo-axis bars simultaneously — topples <=1/6 det AND <=2/6 sto (repeat3/pushacq1 bar) with gait_valid >=10/12 and fault-episodes still limping not statuing (faultobs2 bar), det prog med >=0.9. Prediction-if-false: terms stall in the 2-4/12 band with reward flattening — the joint skill has a higher floor than either solo axis, and the residual is push-during-faulted-stance episodes (check per-episode: do the topples concentrate where the shove hits the faulted side?). Strongest alternative: gait quality erodes (sacrificed legs on NON-faulted episodes) as the policy over-generalizes the limp — that would read as a genuine composition cost hiding under the good termination counts.

**gate**: Acquisition (6M continuation, 8M total, own-cfg DR-0 gate with dr.fault_prob=1.0 AND dr.ext_push_prob=1.0). PASS = topples <=1/6 det AND <=2/6 sto, gait_valid >=10/12 (gv=False allowed only on episodes whose faulted leg is the flagged one), zero sacrificed legs on non-faulted episodes, det prog med >=0.9, video shows limp-and-recover not statue/crouch => push+fault joint skill closed at solo-axis quality; M5 candidate substrate. INFORMATIVE-ceiling = terms 2-4/12 with terms still falling at cutoff (08-21: continue). FAIL = terms worsen vs r2's 2/12 or gait erodes below 9/12 or limp pathology spreads to healthy episodes.

