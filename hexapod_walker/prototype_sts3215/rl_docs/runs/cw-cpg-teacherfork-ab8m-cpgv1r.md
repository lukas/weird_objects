# cw-cpg-teacherfork-ab8m-cpgv1r

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-24T01:09:39+00:00

**pod**: hexapod-mjx-train-4

**steps**: 8000000

**parent**: cw-cpg-teacherfork-ab-cpgv1

**hypothesis**: Does the CPG-searched motion library close its ~15-20% deterministic-margin gap to the scripted teacher when the AMP student trains 4x longer? This is the SECOND adoption data point the cpg track's own STATUS names ('matched larger budget') before any real teacher-swap fork -- funded now that amp is sim-complete and GPU budget allows. Same recipe as the INFORMATIVE 2M A/B cw-cpg-teacherfork-ab-cpgv1 (BC-clone init, section-5 AMP reward, cpg_v1.npz style library, same seed), ONLY steps 2M->8M, read against the budget-matched teacher control cw-cpg-teacherfork-ab8m-teacherr. Prediction-if-true (gap closes): det prog/slip reach parity with the teacher arm at 8M. Prediction-if-false (gap persists/widens): adoption closes at two consistent data points as viable-but-inferior; teacher_v2 stays the style source. Strongest alternative: BOTH arms degrade past a pre-8M optimum -- read at best checkpoints.

**gate**: Matched A/B read vs cw-cpg-teacherfork-ab8m-teacherr (NEVER vs the 2M runs alone, never this arm alone): DR-0 gate det+sto n=6 own cfg, gait_valid >=5/6 both modes, zero sacrificed legs both arms required for any read. ADOPTION-OPEN = this arm's det prog med >= 0.95x the teacher arm's AND slip/m <= 1.10x the teacher arm's, with clean six-leg video both arms. INFORMATIVE-inferior = real walking but still >=10% softer on either axis -> adoption question CLOSES as viable-but-inferior. FAIL = statue/collapse/sacrificed legs. Joint verdict updates the cpg track STATUS's final named item either way.

**verdict**: ADOPTION-OPEN: at the 8M matched-budget read, the CPG-searched motion library (cpg_v1.npz) is parity-or-slightly-better than the scripted teacher (teacher_v2.npz) as the AMP style-reward source, closing this track's second (matched-larger-budget) adoption data point. DR-0 gate n=6 det+sto: this arm det prog med 1.29 vs teacherr's 1.22 (>=0.95x bar met, cpg_v1 actually leads), slip/m 2.68 vs 2.69 (<=1.10x bar met, effectively tied); sto prog med 0.97 vs 0.90 (cpg_v1 ahead), sto slip 3.27 vs 3.27 (tied). Both arms gait_valid 6/6 det+sto, zero terminations, zero sacrificed legs, video-clean six-leg cycling both modes (contact sheets + mp4s in logs/ckpt_eval/cw_cpg_teacherfork_ab8m_cpgv1r_gate/). Read strictly JOINTLY with cw-cpg-teacherfork-ab8m-teacherr per this run's own pre-registered gate -- verdicts cross-reference the same numbers. This is a THIRD independent adoption data point (after the 08-23 8M acq1b/style05-budget2 pair and the 6M ab6m-cpglib/teachlib pair), all pointing the same direction: cpg_v1 is a genuinely co-equal (slightly favored on this pair) AMP style source. No new adoption action beyond what's already recorded -- the track's adoption question was already CLOSED (co-equal, no forced teacher_v2 swap) before this pair landed; this reproduces and strengthens that finding at a third budget/pairing. Track's cpg gate stays GREEN; no remaining named Next items besides maintenance/[operator] physical-robot deployment.

