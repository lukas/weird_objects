# cw-standwalk-stance-mesh2-holdload1min-warm

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-25T08:15:00+00:00

**pod**: hexapod-mjx-train-4

**steps**: 4000000

**parent**: cw-standwalk-stance-mesh2-holdonly1

**hypothesis**: Does repricing rescue the ALREADY-balanced stilt? Warm-start from holdonly1's checkpoint (survives 15s but on three feet) with the load-min hold-income gate ON. holdonly1-acq1 showed continuing WITHOUT repricing destabilizes the stilt into tilt_pitch collapse (0/24); this arm tests whether the load gate instead deforms it into a six-foot plant, which would be far cheaper than from-scratch. Prediction-if-true: six-foot hold within 4M, faster than the scratch pair. Prediction-if-false: same acq1-style destabilization (tilt collapse, declining reward, eval 0/12) -- the stilt is a knife-edge that cannot be locally deformed to a plant; rely on the scratch arms. Strongest alternative: it sheds load partially and parks on the s_i on-ramp -- watch hold_load_factor.

**gate**: Hold panel at 4M: pod_eval hold DR-0 det+sto n=6+6, >=10/12 survive with zero OC/tilt terms, six-foot stance (valid_plant or all-leg duty>=0.9), cur_p95<=1.0A.

