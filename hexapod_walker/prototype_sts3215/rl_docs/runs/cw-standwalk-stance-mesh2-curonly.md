# cw-standwalk-stance-mesh2-curonly

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T07:10:49+00:00

**pod**: hexapod-mjx-train-6

**steps**: 20000000

**parent**: cw-standwalk-stance-mesh2-cur1

**wandb_id**: zwdc1ef5

**hypothesis**: Plain English: cur1 (current_hot pricing + term_cost pricing together) never found ANY stable basin (reward flat negative all 20M, 0/3 seeds), unlike mesh1-rr1 (neither price) which found a profitable-but-wrong grind (reward RISING to +90). Something about the new pricing pair, not just 'pricing exists', blocked all learning. This arm isolates current_hot ALONE (term_cost_per_remaining_s forced to 0, current_hot/current_hot_a unchanged from cur1) to test which of the two new charges is the actual destabilizer -- the complementary ablation to termonly. Prediction-if-current_hot-is-fine-alone: reward climbs like mesh1's grind case (or better, an honest basin) -- term_cost is the destabilizer, drop/soften it. Prediction-if-still-flat: current_hot alone is enough to block learning -- it (not term_cost) is the destabilizer, needs a softer dose or a curriculum ramp.

**gate**: Same stage-1 gate as cur1 at 20M: pod_eval stance panel n>=12 det+sto DR-0+own-DR(0.2). Read jointly with termonly: whichever ablation alone reproduces cur1's flat-negative signature names the destabilizing term; whichever recovers movement (grind or honest) exonerates its own term.

**verdict**: Ablation answer: removing term_cost does NOT rescue the recipe — current_hot pricing alone reproduces cur1's total collapse. 0/36 stance episodes ok (DR-0 gate + own-DR 0.2, det+sto): hold/lower dominated by over_current, rise by tilt; hold_det video shows the body sagging into a hot knee-splayed grind. Training reward flat-negative all 20M (quarters -305/-360/-230/-237) — reward AND gate flat with adequate budget = genuine FAIL per 08-21. Joint read with termonly: all four pricing corners (rr1 neither charge, cur1 both, curonly hot-only, termonly term-only) now fail the from-scratch full goal-mix on mesh/100Hz — pricing is not the blocking lever, the multi-task diet is (holdonly1 proved balance IS learnable hold-only). Rung 3 goes hold-first with the load-gated hold income.

