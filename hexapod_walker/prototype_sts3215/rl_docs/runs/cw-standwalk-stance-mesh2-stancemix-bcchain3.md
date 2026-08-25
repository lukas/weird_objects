# cw-standwalk-stance-mesh2-stancemix-bcchain3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T13:33:56+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdminload40-bcanchor3

**wandb_id**: tj2k8oxo

**hypothesis**: Does the imitation-anchor bundle survive the full stance diet (hold 10% / rise 45% / lower 45% -- the exact goal-mix the stage-1 gate needs and the one that collapsed EVERY pure-reward mesh recipe in rungs 1-6)? Rung-8 composition read, sibling of riseonly/loweronly-bcchain3: same mesh bcanchor3 recipe + footlow2-PASS anchor bundle (per-mode anchors: q_nom hold, state-aligned rise chain, IK-descent lower chain -- this per-mode supervision is exactly what the primitive-era footlow2 mix PASSed with), all three modes at once. Prediction-if-true: no mode pins at zero by 2M -- hold keeps the six-foot plant (>=4/6 det valid_plant) while rise and lower show visible progress; the rung-3 'goal-mix structure' collapse was really 'no target signal', which anchors supply. Prediction-if-false: the mix collapse reappears even WITH per-mode anchors (modes fight in one network at this scale) -- then stage-1 proceeds mode-isolated (rise/lower acquisitions off the isolated canaries) and unification happens at stage-2 distillation instead of via goal-mix. Strongest alternative: hold survives but rise/lower learn 4x slower than their isolated siblings (dilution, not collapse) -- then this arm just needs the acquisition budget, judge against the isolated arms' 2M curves.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition or close a behavior/reward class at this checkpoint. 2M, DR-0 det panel: PASS = hold >=4/6 valid_plant retained AND (rise OR lower) showing nonzero honest progress (any det episode ending valid_plant for rise / >=60% commanded drop feet-planted for lower, or clearly rising in-training success), zero over_current pins; PARTIAL = hold retained but rise+lower both still zero with reward rising (dilution -- compare the isolated siblings before funding); FAIL = hold collapses too (mix-collapse signature reappears despite anchors) -> mode-isolated stage-1 + stage-2 distillation fork.

