# cw-amp-m2-pilot-noamp

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-22T10:36:16+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**wandb_id**: vabw04c5

**hardware_ready**: no

**hypothesis**: Matched no-AMP control for cw-amp-m2-pilot-style05: identical from-scratch config (asym-critic, brief §6 stress_mix envelope, seed 7, 40M) minus every amp flag - isolates what the style reward itself buys. This is also the first SCORED long run of the v3 smoke's task-only config (the 500k smoke was non-scoring). Prediction: without a motion prior the policy finds SOME locomotion but with the usual from-scratch pathologies (paddle-creep/dragging/non-tripod contact patterns) that the AMP arm should visibly beat; if instead this control is already clean, the style reward must justify itself on smoothness/style metrics or the wave-1 AMP weights need rethinking.

**gate**: Control arm: judged only as the comparison baseline for cw-amp-m2-pilot-style05 (videos + per-leg gait metrics + falls at equal budget). No SKILLS/champion updates. If BOTH arms fail to locomote at all, branch (iii) of the pilot hypothesis fires: narrow the command envelope (brief §6 curriculum 30-50% start) before touching AMP knobs.

**verdict**: Control arm of the M2 pilot pair, judged only vs cw-amp-m2-pilot-style05 at equal budget: at 2M (~1.3 episodes/env) both arms are pre-locomotion (det fwd 0.026m/15s, slip ~9.7-11.4/m, 2/6 det gait-valid with sacrificed legs; sto 6/6 gait-valid but fwd 0.099m; 0 falls; contact sheet = sprawled near-frozen stance, same pathology class as the style arm). No informative style-vs-control delta this early; branch (iii) NOT declared — too little budget to call the envelope binding. CONTINUING matched to 40M total (respec -c1).

