# amp-m2-pilot-noamp

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-22T10:34:28+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**hypothesis**: Matched no-AMP control for amp-m2-pilot-style05: identical from-scratch config (asym-critic, brief §6 stress_mix envelope, seed 7, 40M) minus every amp flag - isolates what the style reward itself buys. This is also the first SCORED long run of the v3 smoke's task-only config (the 500k smoke was non-scoring). Prediction: without a motion prior the policy finds SOME locomotion but with the usual from-scratch pathologies (paddle-creep/dragging/non-tripod contact patterns) that the AMP arm should visibly beat; if instead this control is already clean, the style reward must justify itself on smoothness/style metrics or the wave-1 AMP weights need rethinking.

**gate**: Control arm: judged only as the comparison baseline for amp-m2-pilot-style05 (videos + per-leg gait metrics + falls at equal budget). No SKILLS/champion updates. If BOTH arms fail to locomote at all, branch (iii) of the pilot hypothesis fires: narrow the command envelope (brief §6 curriculum 30-50% start) before touching AMP knobs.

**refused_reason**: experiments must use the cw- prefix

