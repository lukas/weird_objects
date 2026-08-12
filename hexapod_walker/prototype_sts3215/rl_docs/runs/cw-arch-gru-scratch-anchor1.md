# cw-arch-gru-scratch-anchor1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-12T01:42:51+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-arch-gru-r4c

**wandb_id**: koc1aon1

**hardware_ready**: False

**hypothesis**: gru-r1..r4c: EVERY from-scratch GRU at 2M finds the leg-sacrifice paddle before finding gait, surviving the full anti-cheat stack — an optimization failure (nothing tells a churning leg WHICH WAY to move), the exact signature probe_walk_income exonerated pricing for. The BC anchor supplies that missing gradient in action space (TripodGait walk targets, state-aligned rise refs, settled-pose hold, IK lower; stratified quotas), is validated on the stand track (loweranchor1 2/6 -> 6/6) and is live-converging in the recurrent form on cw-arch-gru-anchor1 right now (anchor loss 0.020 -> 0.005 by 6.5M with KL calm). If action supervision from tick zero beats the paddle attractor, a from-scratch GRU learns honest gait AND stance in one run with no distillation stage at all; if the paddle still wins with a live converged anchor, the paddle is not a missing-gradient artifact and the from-scratch line closes for good.

**gate**: 2M forensics det+sto @DR0 r3 gate cfg vs the frozen r1-r4c fingerprint: PASS if NO leg-sacrifice paddle (zero parked legs det, all six feet cycle) AND det walk gait_valid >=4/6 AND hold det >=3/6 AND train/bc_anchor_loss converged below 0.02 — then respec the SAME arm to 10M+ hardening citing this as evidence. FAIL if any det episode shows the r1-r4c parked-leg fingerprint (paddle beats a live anchor -> from-scratch line CLOSED, distill+finetune remains the only GRU path) OR anchor loss plateaus >0.05 (targets unreachable from random init -> anchor needs curriculum, not more steps).

**verdict**: FAIL (known exploit, STOP) -- the from-scratch GRU with a live, well-converged anchor (train/bc_anchor_loss 0.010, below the 0.02 threshold) STILL loses to the parked-leg paddle: det walk gait_valid 0/6, leg idx1 explicitly flagged sacrificed, zero net travel. This is exactly the pre-registered FAIL branch (paddle beats a live anchor). Per this run's own pre-registration, the from-scratch-GRU+anchor line is CLOSED -- distill-then-finetune (cw-arch-gru-bc-ft1, warm from a BC-distilled net) remains the only path that keeps a GRU walking cheat-free; a live anchor cannot rescue a random init the way it rescues a warm start.

