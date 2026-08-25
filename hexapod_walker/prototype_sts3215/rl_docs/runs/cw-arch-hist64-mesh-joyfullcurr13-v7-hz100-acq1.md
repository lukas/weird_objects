# cw-arch-hist64-mesh-joyfullcurr13-v7-hz100-acq1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-25T01:27:08+00:00

**pod**: hexapod-mjx-train-7

**steps**: 38000000

**parent**: cw-arch-hist64-mesh-joyfullcurr13-v7-hz100-canary1

**hypothesis**: Plain English: both mesh-family 2M mechanism-health canaries (this MLP clone + the tf64 transformer sibling) passed clean -- now give the MLP-mesh recipe the full 40M budget so it becomes the mesh family's own valley/acquisition reference, the exact role cw-arch-hist64-joyfullcurr13-v7-hz100-scratch-s0-r1 played for the primitive family. Continues this canary's checkpoint 38M more steps (40M total) on the identical V7/hist64/100Hz/mesh stack. Prediction-if-true: reward follows the same architecture-independent from-scratch valley shape already seen on the primitive family (deep trough ~5-10M, crossing zero ~12-16M, positive and frontier-promoting past b0 by 40M) -- possibly deeper/later given the mesh model's +66% mass. Prediction-if-false: reward stays flat/non-improving well past the point the primitive-family MLP recovered (15M+) with frontier pinned at b0 -- a genuine mesh-dynamics learnability problem, not a valley. Strongest alternative: the heavier mesh model shifts the valley deeper/longer without being unhealthy -- read by shape/turn timing against the primitive MLP's own trajectory (not exact values, different dynamics), and this run's OWN waypoints become the reference the tf64-mesh acquisition is judged against.

**gate**: PASS: frontier promotions past b0 with reward/eval AGREEMENT, plus the standard 60s randomized joygate at 100Hz on the mesh model (falls<=2/48, directions followed, slip<=~2.9/m, video shows all six feet cycling -- explicit per-leg duty tracking for the legs-3/5-or-other sacrifice fingerprint given 3 prior primitive-family lineages showed it). Record 2M/7M/12M/20M/40M ep_rew_mean waypoints in the run doc as the mesh-family valley reference (no such reference exists yet). PARTIAL: genuinely learning (b0+ promotions, improving rung evals) but short of the primitive-family MLP's own matched-step shape by 40M -- continuation candidate per 08-21 (expected: heavier mass may need more budget). FAIL: b0 never promotes with reward AND rung metrics flat at adequate budget (>=20M), or reward rises while rungs stay flat (reward/eval mismatch -- audit before any seed sweep). This run's own trajectory becomes the control cw-arch-tf64-mesh-joyfullcurr13-v7-hz100-acq1 is judged against.

**refused_reason**: acquisition runs require --evidence: name the healthy canary and a comparable full-budget learning precedent.

