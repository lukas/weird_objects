# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-26T16:57:01+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor1

**wandb_id**: eytblwvv

**hypothesis**: Plain English: the stance-only BC anchor was wrecking walking not through its gradients (those are exactly zero on walk ticks) but through the shared Adam optimizer's MOMENTUM -- every aux minibatch also 'stepped' the gated-out walk core on stale PPO momentum (8 extra uncommanded steps per update; dig-in on -anchor1/-anchor1-s1, probe: walk path |dParam| 0.815 with |grad| exactly 0; unit tests pin defect+fix, commit 2f585a97). This arm is the anchor1 recipe with exactly ONE change: train.bc_anchor_isolate_update=1 drops populated all-zero grads before the aux optimizer step, so a stance-only anchor batch leaves the walk actor path bit-identical. Prediction-if-true: walk returns to at least the bare fine-tune's weak crawl (modeseq1 prog_ratio 0.19-0.38 det DR-0) with NO new freeze/shuffle catastrophe, while hold/lower keep at least anchor1's det-side recovery (5/6, 4/6) and plausibly reach isolated failure now that the anchor's effect is clean. Prediction-if-false (walk still catastrophically frozen/shuffling): the momentum leak was not the walk-wrecker -- strongest alternative is the SAME zero-grad momentum channel inside PPO's own update (single-family recurrent minibatches under goal.mode_seq=0.75) or cross-mode value mixing, escalate to that audit before any further anchor arm.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition or close a behavior/reward class at 2M. Joint call with -anchor2-s1. LEAK-FIX PASS if on BOTH seeds walk shows NO anchor1-class catastrophe (no 5-leg sacrifice freeze, no negative-prog high-slip shuffle; prog_ratio det DR-0 >= ~0.2, i.e. back in the bare fine-tune's own band or better). FULL PASS additionally requires hold AND lower collapsing to isolated failure (<=1/6 term/fail each of det+sto, both DR) on both seeds -- the original anchor promise, now measured without the leak confounder. FAIL-A (walk still wrecked both seeds): momentum leak was not the walk mechanism -> audit PPO-side single-family-minibatch momentum + value mixing; do NOT fund another anchor dose. FAIL-B (walk fixed but hold/lower still majority-fail sto): the leak explanation stands but the stance anchor at coef=3.0 cannot rescue mesh hold-sto -> next lever is the stance teacher/dose, not routing. Read reward trend per the 08-21 ruling before any verdict.

