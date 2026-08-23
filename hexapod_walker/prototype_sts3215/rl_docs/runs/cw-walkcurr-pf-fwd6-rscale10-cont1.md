# cw-walkcurr-pf-fwd6-rscale10-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-23T20:56:46+00:00

**pod**: hexapod-mjx-train-0

**steps**: 4000000

**parent**: cw-walkcurr-pf-fwd6-rscale10

**hypothesis**: Plain English: the x0.1 reward-scale arm finally got PPO's optimizer unstuck, but only in the last ~15% of its 2M budget -- give the SAME run 4M more steps to see if walking behavior actually starts moving now that learning is mechanically alive. Parent cw-walkcurr-pf-fwd6-rscale10 ended with clip_fraction recovered and RISING (0.021@1632k -> 0.079@2016k, never exactly 0 -- unique in the fwd6 wave; gru/sde both collapsed to 0 and froze, completing the dig-in's cross-prediction), value_loss 8-18 vs 400-2400 unscaled, approx_kl 0.02, std creeping off init -- but eval behavior still the static crouch (gate 0/6 det, freeprog hovering -0.05..-0.07). ~300k effective post-recovery optimization steps is too few to expect gait discovery; this is the 08-21 continue branch of the parent's own pre-registered mechanism-health gate. Byte-identical config + --init-from parent checkpoint (single lever: budget); note the 1M-step charge ramp (min_frac 0.4) restarts with the continuation -- bank-proven pricing, and re-softened charges during renewed exploration can only help discovery. If-true: walk_freeprog_score leaves the [-0.10,-0.05] band and trends toward 0, det gate shows nonzero prog. If-false (freeprog flat ANOTHER 4M with clip_fraction staying healthy >0.02): optimizer-crush was necessary but NOT sufficient -- advantage-starvation/exploration is the residual blocker, escalate to RND state-novelty or rung-0 sub-goal per STATUS, no further same-recipe continuations. Strongest alternative: sibling rscale50 (x0.02) un-crushes from step 0 and answers faster.

**gate**: Rung-1 gate (same as fwd1-fwd6): C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes, video shows real stepping. Mechanism-health at +4M: freeprog_score > -0.02 or clearly risen out of [-0.10,-0.05] = discovery started (continue/iterate); freeprog still flat in band WITH clip_fraction healthy (>0.02 majority of run) = optimizer-crush necessary-but-insufficient, escalate to RND/rung-0, same-recipe lineage closed.

**refused_reason**: discovery runs cap at 2000000 steps (asked 4000000): the question is 'did qualitatively correct behavior emerge?' - continue as --phase hardening with --evidence.

