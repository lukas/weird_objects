# cw-arch-gru-anchor2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-12T03:06:45+00:00

**pod**: hexapod-mjx-train-0

**steps**: 10000000

**parent**: cw-arch-gru-anchor1

**wandb_id**: dgndq00g

**hardware_ready**: False

**hypothesis**: Teach the GRU to stand/hold/rise via demonstration while letting reward alone teach it to walk, since teaching walking by demonstration froze it solid. Same recipe as cw-arch-gru-anchor1 (warm from the ft1 BC-distilled GRU, identical rise/hold/lower anchor stack, identical goal-mix and reward cfg) with ONE variable: the new train.bc_anchor_walk=0 flag turns OFF the walk-tick anchor while leaving rise/hold/lower anchored. Prediction-if-true: walk recovers to gait_valid>=5/6 with real progress (prog_ratio>=0.80, no freeze) while hold/lower/rise keep anchor1's gains. Prediction-if-false: walk still fails (e.g. erodes back toward the ft1/ft2 hold-forgetting pattern even without a walk anchor) -- meaning the anchor's gradient on OTHER modes is itself interfering with walk, not just the walk-anchor term, and the next lever is per-mode loss weighting, not a binary on/off.

**gate**: PASS if det walk gait_valid>=5/6 AND prog_ratio med>=0.80 (no freeze/paddle) AND hold det>=4/6 AND lower det>=4/6 AND rise det>=2/6 with >=1 non-flat start. FAIL if walk still freezes/paddles (anchor1's problem persists even with the walk anchor off -> the interference is cross-mode, not the walk term itself) OR hold/lower erode below anchor1's levels (removing the walk anchor changed the training dynamics enough to hurt the modes it was supposed to protect).

**verdict**: FAIL, informatively -- confirms the pre-registered false branch. Dropping just the walk-tick anchor (train.bc_anchor_walk=0) did NOT fix the freeze: det walk gait_valid reads 6/6 but prog_ratio 0.01, speed 0.002 m/s, slip/m 4.53, video pixel-static across all 10 sampled frames both det+sto (identical fingerprint to cw-arch-gru-anchor1). Hold/lower held anchor1's gains (det 6/6 each, drag 103/152mm); rise regressed to 1/6 det (bridge 0/3, crouch 1/1, flat 0/2) vs anchor1's 2/6, below the >=2/6 gate bar. Proves the interference is the SHARED recurrent trunk (feature extractor + GRU cell every mode's forward pass shares), not the walk-anchor loss term itself. Two consecutive misses on the binary on/off anchor lever = mechanism change (RESEARCH_RULES). Follow-up cw-arch-gru-anchor3 (train.bc_anchor_detach_trunk=1, new CODE, default off/bit-exact, tests green) stops the anchor gradient at the GRU output so stance-tick anchoring only trains the actor head, never the shared trunk.

