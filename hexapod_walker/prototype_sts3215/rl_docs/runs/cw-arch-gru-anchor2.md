# cw-arch-gru-anchor2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-12T03:06:45+00:00

**pod**: hexapod-mjx-train-0

**steps**: 10000000

**parent**: cw-arch-gru-anchor1

**wandb_id**: dgndq00g

**hypothesis**: Teach the GRU to stand/hold/rise via demonstration while letting reward alone teach it to walk, since teaching walking by demonstration froze it solid. Same recipe as cw-arch-gru-anchor1 (warm from the ft1 BC-distilled GRU, identical rise/hold/lower anchor stack, identical goal-mix and reward cfg) with ONE variable: the new train.bc_anchor_walk=0 flag turns OFF the walk-tick anchor while leaving rise/hold/lower anchored. Prediction-if-true: walk recovers to gait_valid>=5/6 with real progress (prog_ratio>=0.80, no freeze) while hold/lower/rise keep anchor1's gains. Prediction-if-false: walk still fails (e.g. erodes back toward the ft1/ft2 hold-forgetting pattern even without a walk anchor) -- meaning the anchor's gradient on OTHER modes is itself interfering with walk, not just the walk-anchor term, and the next lever is per-mode loss weighting, not a binary on/off.

**gate**: PASS if det walk gait_valid>=5/6 AND prog_ratio med>=0.80 (no freeze/paddle) AND hold det>=4/6 AND lower det>=4/6 AND rise det>=2/6 with >=1 non-flat start. FAIL if walk still freezes/paddles (anchor1's problem persists even with the walk anchor off -> the interference is cross-mode, not the walk term itself) OR hold/lower erode below anchor1's levels (removing the walk anchor changed the training dynamics enough to hurt the modes it was supposed to protect).

