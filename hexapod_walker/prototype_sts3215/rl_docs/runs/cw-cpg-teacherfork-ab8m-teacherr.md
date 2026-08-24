# cw-cpg-teacherfork-ab8m-teacherr

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-24T01:13:03+00:00

**pod**: hexapod-mjx-train-5

**steps**: 8000000

**parent**: cw-amp-m2-bcinit-sec5-style05

**wandb_id**: fpop3lx6

**hypothesis**: Control arm for the CPG-library adoption A/B at 4x budget: does the proven scripted-teacher style recipe simply get better with more training, and what do its margins look like at 8M? Same recipe as the 2M PASS cw-amp-m2-bcinit-sec5-style05 (BC-clone init, section-5 AMP reward, teacher_v2.npz style library, same seed), ONLY steps 2M->8M. This gives the cpg_v1 treatment arm (cw-cpg-teacherfork-ab8m-cpgv1r) a budget-matched baseline -- the cpg track's second adoption data point. Prediction-if-true: margins hold or improve modestly over its own 2M read (det prog med >=1.16, slip not worse, gait_valid 6/6). Prediction-if-false (degrades at 8M): the sec5 style reward has an optimum before 8M -- read at each arm's best checkpoint, degradation is a finding about the reward not the library. Strongest alternative: no change beyond noise (recipe saturates by 2M).

**gate**: DR-0 gate det+sto n=6 at own cfg: gait_valid >=5/6 both modes, ZERO sacrificed legs, det prog med >= 1.16 minus eval noise, clean six-leg video. This is the BASELINE side of a matched pair -- verdict written JOINTLY with cw-cpg-teacherfork-ab8m-cpgv1r (never read either arm alone); the pair's decision rule lives in the cpgv1r arm's gate.

