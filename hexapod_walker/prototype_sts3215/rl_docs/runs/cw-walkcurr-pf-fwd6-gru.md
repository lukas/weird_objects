# cw-walkcurr-pf-fwd6-gru

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T20:33:11+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd3-chargeramp

**wandb_id**: pu0p72ur

**hypothesis**: Plain English: fwd1-fwd5 (8 arms) all froze in an identical static crouch under a memoryless MLP actor, and a W&B-history read confirmed train/clip_fraction collapses to exactly 0 early and never recovers regardless of reward pricing or exploration-noise magnitude. This arm tests the one architecture-side candidate named in STATUS.md's own Next list that hasn't been implicated by any finding yet but also hasn't been tried: swap the from-scratch actor/critic for the in-repo recurrent GRU path (--gru, sb3-contrib RecurrentPPO). A recurrent hidden state carries information across the whole BPTT window (--n-steps 64, the documented recommended length for --gru vs the MLP recipe's 24, so the memory actually gets a usable window -- widening n-steps is part of the SAME single lever, not a second one, since --gru is defined to want it), which could let the critic/actor represent progress-since-episode-start rather than only current state+time, potentially avoiding the frozen-policy's near-zero-advantage trap the vanishing-gradient diagnosis describes. Fresh init, 2M steps, everything else byte-identical (charge ramp kept). Prediction-if-true: env/walk_freeprog_score leaves its flat [-0.10,-0.05] band and trends toward/above 0. Prediction-if-false: identical frozen video/flat score -- memorylessness is exonerated (matches the diagnosis's own framing: the trap is a property of the STATE-reward relationship, not the architecture), closing this STATUS.md Next-list item and sharpening the case for rung-0/RND.

**gate**: Rung-1 gate (same as fwd1-fwd5): C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes, video shows real stepping. Discovery-health (08-21 ruling): env/walk_freeprog_score > -0.02 and rising at 2M = continue; still flat in [-0.10,-0.05] = recurrence hypothesis refuted, no same-recipe continuation -- escalates to rung-0/RND.

