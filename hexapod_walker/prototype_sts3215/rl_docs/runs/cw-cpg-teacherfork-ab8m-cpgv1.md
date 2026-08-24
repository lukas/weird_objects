# cw-cpg-teacherfork-ab8m-cpgv1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-24T00:47:01+00:00

**pod**: hexapod-mjx-train-1

**steps**: 8000000

**parent**: cw-cpg-teacherfork-ab-cpgv1

**hypothesis**: Does the CPG-searched motion library close its ~15-20% deterministic-margin gap to the scripted teacher when the AMP student trains 4x longer? This is the SECOND adoption data point the cpg track's own STATUS names ('matched larger budget') before any real teacher-swap fork — funded now that amp is sim-complete and GPU budget allows. Same recipe as the INFORMATIVE 2M A/B cw-cpg-teacherfork-ab-cpgv1 (BC-clone init, section-5 AMP reward, cpg_v1.npz style library, same seed), ONLY steps 2M->8M, read against the budget-matched teacher control cw-cpg-teacherfork-ab8m-teacher. Prediction-if-true (gap closes): det prog/slip reach parity with the teacher arm at 8M — the 2M gap was a style-source convergence-speed artifact, and the adoption fork opens. Prediction-if-false (gap persists/widens): adoption closes at two consistent data points as viable-but-inferior; teacher_v2 stays the style source and the cpg track's last named item is settled. Strongest alternative: BOTH arms degrade past a pre-8M optimum — read at best checkpoints, conclusion limited to 'no budget-driven reordering'.

**gate**: Matched A/B read vs cw-cpg-teacherfork-ab8m-teacher (NEVER vs the 2M runs alone, never this arm alone): DR-0 gate det+sto n=6 own cfg, gait_valid >=5/6 both modes, zero sacrificed legs both arms required for any read. ADOPTION-OPEN = this arm's det prog med >= 0.95x the teacher arm's AND slip/m <= 1.10x the teacher arm's, with clean six-leg video both arms. INFORMATIVE-inferior = real walking but still >=10% softer on either axis -> adoption question CLOSES as viable-but-inferior (two data points, no swap). FAIL = statue/collapse/sacrificed legs. Joint verdict updates the cpg track STATUS's final named item either way.

**refused_reason**: acquisition runs require --evidence: name the healthy canary and a comparable full-budget learning precedent.

