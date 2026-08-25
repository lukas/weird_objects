# cw-standwalk-stance-mesh2-curonly

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-25T07:07:48+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-standwalk-stance-mesh2-cur1

**hypothesis**: Plain English: cur1 (current_hot pricing + term_cost pricing together) never found ANY stable basin (reward flat negative all 20M, 0/3 seeds), unlike mesh1-rr1 (neither price) which found a profitable-but-wrong grind (reward RISING to +90). Something about the new pricing pair, not just 'pricing exists', blocked all learning. This arm isolates current_hot ALONE (term_cost_per_remaining_s forced to 0, current_hot/current_hot_a unchanged from cur1) to test which of the two new charges is the actual destabilizer. Prediction-if-current_hot-is-fine-alone: reward climbs like mesh1's grind case (or better, an honest basin) -- term_cost is the destabilizer, drop/soften it. Prediction-if-still-flat: current_hot alone is enough to block learning -- it (not term_cost) is the destabilizer, needs a softer dose or a curriculum ramp.

**gate**: Same stage-1 gate as cur1 at 20M: pod_eval stance panel n>=12 det+sto DR-0+own-DR(0.2). Informative either way: PASS-comparable is >=5/6 rise/lower + 6/6 hold; a reward trajectory that leaves cur1's flat/negative band (even short of the full gate) still answers the ablation question.

**refused_reason**: hexapod-mjx-train-3 already runs cw-standwalk-stance-mesh2-cur1-reftrack10 — GPU pods host exactly one run; pick a free GPU pod.

