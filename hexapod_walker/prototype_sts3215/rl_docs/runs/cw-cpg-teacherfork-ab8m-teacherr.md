# cw-cpg-teacherfork-ab8m-teacherr

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-24T01:13:03+00:00

**pod**: hexapod-mjx-train-5

**steps**: 8000000

**parent**: cw-amp-m2-bcinit-sec5-style05

**wandb_id**: fpop3lx6

**hypothesis**: Control arm for the CPG-library adoption A/B at 4x budget: does the proven scripted-teacher style recipe simply get better with more training, and what do its margins look like at 8M? Same recipe as the 2M PASS cw-amp-m2-bcinit-sec5-style05 (BC-clone init, section-5 AMP reward, teacher_v2.npz style library, same seed), ONLY steps 2M->8M. This gives the cpg_v1 treatment arm (cw-cpg-teacherfork-ab8m-cpgv1r) a budget-matched baseline -- the cpg track's second adoption data point. Prediction-if-true: margins hold or improve modestly over its own 2M read (det prog med >=1.16, slip not worse, gait_valid 6/6). Prediction-if-false (degrades at 8M): the sec5 style reward has an optimum before 8M -- read at each arm's best checkpoint, degradation is a finding about the reward not the library. Strongest alternative: no change beyond noise (recipe saturates by 2M).

**gate**: DR-0 gate det+sto n=6 at own cfg: gait_valid >=5/6 both modes, ZERO sacrificed legs, det prog med >= 1.16 minus eval noise, clean six-leg video. This is the BASELINE side of a matched pair -- verdict written JOINTLY with cw-cpg-teacherfork-ab8m-cpgv1r (never read either arm alone); the pair's decision rule lives in the cpgv1r arm's gate.

**verdict**: Baseline arm of the pre-registered 8M matched-budget CPG-vs-teacher adoption pair PASSES its own bar: DR-0 det prog med 1.22 (>=1.16 bar), sto prog med 0.90, gait_valid 6/6 both modes, zero sacrificed legs, zero terminations, slip/m 2.69 det / 3.27 sto (inside 1.4-2.9-ish teacher band), video-clean six-leg cycling both modes. Read JOINTLY with cw-cpg-teacherfork-ab8m-cpgv1r per this run's own gate text: cpgv1r det prog med 1.29 (>=0.95x this arm's 1.22 -> ADOPTION-OPEN met, in fact cpg_v1 slightly LEADS), slip/m 2.68 (<=1.10x this arm's 2.69 -> met), sto prog 0.97 vs 0.90 (cpg_v1 ahead), sto slip 3.27 vs 3.27 (tied); both arms gait_valid 6/6, zero terms/sacrificed, video-clean. This is a THIRD independent adoption data point (after the 08-23 8M acq1b/style05-budget2 pair and the 6M ab6m-cpglib/teachlib pair) and it reproduces the same result: cpg_v1 is parity-or-slightly-better than teacher_v2 as an AMP style source. No new action -- the track's adoption question was already CLOSED (co-equal, no forced swap) before this pair landed; this just adds confirming evidence. Evidence: logs/ckpt_eval/cw_cpg_teacherfork_ab8m_teacherr_gate/, logs/ckpt_eval/cw_cpg_teacherfork_ab8m_cpgv1r_gate/.

