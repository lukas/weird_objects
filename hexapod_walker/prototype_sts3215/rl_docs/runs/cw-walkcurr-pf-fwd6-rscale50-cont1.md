# cw-walkcurr-pf-fwd6-rscale50-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-23T21:08:46+00:00

**pod**: hexapod-mjx-train-1

**steps**: 4000000

**parent**: cw-walkcurr-pf-fwd6-rscale50

**hypothesis**: Plain English: the x0.02 reward-scale run is the first arm whose walking score improved steadily for its whole 2M budget -- give the same policy 4M more steps to see if it crosses from 'drifting less' into actual stepping. Parent cw-walkcurr-pf-fwd6-rscale50 ended with clip_fraction healthy every quarter (means 0.024-0.050, last 0.083; never the exactly-0 collapse of the 9 frozen rung-1 arms), std creeping off init (0.3678->0.3729), and walk_freeprog_score rising monotonically -0.103->-0.015 -- nearly at the zero crossing where forward progress starts paying positive score; eval behavior still the static splayed crouch (gate 0/6 det, fwd 0.01m). Single lever: budget (byte-identical cfg minus the from-scratch-only --activation-fn flag, warm-start keeps the checkpoint's own elu; the 1M-step charge ramp min_frac 0.4 restarts with the continuation -- bank-proven pricing, re-softened charges during renewed exploration can only help discovery). This is the 08-21 continue branch of the parent's own pre-registered mechanism-health gate, mirroring sibling rscale10-cont1; dose note: rscale50's freeprog trend strictly dominates rscale10's flat -0.055 band, refuting the overshoot alternative in the parent's hypothesis. If-true: walk_freeprog_score crosses 0 and keeps rising, det gate shows nonzero prog/real stepping. If-false (freeprog plateaus below 0 across +4M with clip_fraction staying >0.02): the optimizer-crush fix was necessary but NOT sufficient at either dose -- fire the pre-registered escalation (RND state-novelty bonus or rung-0 sub-goal), no further same-recipe continuations. Strongest alternative: the parent's rise reflects learning stiller postures (fewer negative-progress excursions) rather than approach to walking -- the zero-crossing demand discriminates: stillness saturates just below 0, walking crosses it.

**gate**: Rung-1 gate (same as fwd1-fwd6): C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes, video shows real stepping. Continuation-health at 6M total: walk_freeprog_score crosses 0 and keeps rising = discovery live (continue or respec next rung); freeprog plateaued below 0 over the final 2M with clip_fraction >0.02 = crush-fix insufficient, escalate to RND/rung-0 per track STATUS, no further same-recipe continuations.

