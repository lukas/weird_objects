# cw-standwalk-stance-mesh2-termonly

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T07:07:49+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-standwalk-stance-mesh2-cur1

**wandb_id**: vbs2jyh1

**hypothesis**: Plain English: cur1 (current_hot pricing + term_cost pricing together) never found ANY stable basin (reward flat negative all 20M, 0/3 seeds), unlike mesh1-rr1 (neither price) which found a profitable-but-wrong grind (reward RISING to +90). This arm isolates term_cost_per_remaining_s ALONE (k_current_hot forced to 0, term_cost unchanged from cur1) -- the complementary ablation to curonly -- to test which of the two new charges is the actual destabilizer. Prediction-if-term_cost-is-fine-alone: reward climbs (grind returns since current_hot pricing is what killed it, or an honest basin if term_cost alone is enough deterrent) -- current_hot is the destabilizer, needs a softer dose. Prediction-if-still-flat: term_cost alone blocks learning -- it (not current_hot) is the destabilizer, needs a softer per-second rate or lower cap.

**gate**: Same stage-1 gate as cur1 at 20M: pod_eval stance panel n>=12 det+sto DR-0+own-DR(0.2). Read jointly with curonly: whichever ablation alone reproduces cur1's flat-negative signature names the destabilizing term; whichever recovers movement (grind or honest) exonerates its own term.

