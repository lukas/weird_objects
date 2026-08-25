# cw-standwalk-stance-mesh2-stancemix-bcchain3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PARTIAL

**created**: 2026-08-25T13:33:56+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdminload40-bcanchor3

**wandb_id**: tj2k8oxo

**hypothesis**: Does the imitation-anchor bundle survive the full stance diet (hold 10% / rise 45% / lower 45% -- the exact goal-mix the stage-1 gate needs and the one that collapsed EVERY pure-reward mesh recipe in rungs 1-6)? Rung-8 composition read, sibling of riseonly/loweronly-bcchain3: same mesh bcanchor3 recipe + footlow2-PASS anchor bundle (per-mode anchors: q_nom hold, state-aligned rise chain, IK-descent lower chain -- this per-mode supervision is exactly what the primitive-era footlow2 mix PASSed with), all three modes at once. Prediction-if-true: no mode pins at zero by 2M -- hold keeps the six-foot plant (>=4/6 det valid_plant) while rise and lower show visible progress; the rung-3 'goal-mix structure' collapse was really 'no target signal', which anchors supply. Prediction-if-false: the mix collapse reappears even WITH per-mode anchors (modes fight in one network at this scale) -- then stage-1 proceeds mode-isolated (rise/lower acquisitions off the isolated canaries) and unification happens at stage-2 distillation instead of via goal-mix. Strongest alternative: hold survives but rise/lower learn 4x slower than their isolated siblings (dilution, not collapse) -- then this arm just needs the acquisition budget, judge against the isolated arms' 2M curves.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition or close a behavior/reward class at this checkpoint. 2M, DR-0 det panel: PASS = hold >=4/6 valid_plant retained AND (rise OR lower) showing nonzero honest progress (any det episode ending valid_plant for rise / >=60% commanded drop feet-planted for lower, or clearly rising in-training success), zero over_current pins; PARTIAL = hold retained but rise+lower both still zero with reward rising (dilution -- compare the isolated siblings before funding); FAIL = hold collapses too (mix-collapse signature reappears despite anchors) -> mode-isolated stage-1 + stage-2 distillation fork.

**verdict**: CANARY PASS (partial — mechanism healthy, every mode degraded vs its isolated sibling). The three-mode stance mix keeps all three skills alive at once for the first time on the 3.5kg mesh robot, but every skill is a hot, degraded copy of its mode-isolated sibling. DR-0 det evidence (2M canary, gate report): hold stays planted (valid_plant 6/6, h_err 8.7mm) but EVERY episode trips hold_min_load ~8s in — a load-shedding shuffle (slip 1.6m, duty 0.74-0.88) running HOT at cur_max 2.62A vs the hold champion's 0.44A; rise gets its first honest in-mix det successes (2/6, h_err 3-8mm) with 2 over_current terms; lower det 0/6 (stalls 18-32mm above the crouch) while lower sto descends 4/6 — noise dithers it down, det policy refuses. Own-DR similar (hold det 4/6 survive, hold sto 0/6 min_load). Vs the pre-registered clauses: hold retained + rise honest progress = the PASS shape, but 'zero over_current pins' fails (current pinned ~2.64A nearly everywhere) => partial, not fundable as-is at acquisition scale. Mechanism read: the footlow2 anchor bundle DOES prevent the rung-1-6 mix collapse — but from-scratch mix at 2M buys three hot half-skills instead of one good one (dilution + interference: loweronly det was 6/6 clean at the same budget). Footlow2 comparison the operator asked for (fb_20260825T140238_d43b35): the primitive footlow2 mix runs (hard1/-s1/-stable1) were NEVER from-scratch — every one warm-started an already-competent stance policy (r1 lineage) and PASSed as mix HARDENING; from-scratch mix was never the proven recipe. Next: warm-mix canary pair (footlow2raw18-mesh2-hz100-warmmix1/-warmmix2-lowstd: exact mix recipe init-from the mesh hold stdanneal champion, +/- warm-log-std-override) to reproduce footlow2's real structure on mesh; the mode-isolated rise/lower stdanneal acquisitions stay the primary stage-1 path.

