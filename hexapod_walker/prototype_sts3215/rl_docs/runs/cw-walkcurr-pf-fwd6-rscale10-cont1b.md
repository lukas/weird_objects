# cw-walkcurr-pf-fwd6-rscale10-cont1b

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T21:05:06+00:00

**pod**: hexapod-mjx-train-0

**steps**: 4000000

**parent**: cw-walkcurr-pf-fwd6-rscale10

**wandb_id**: jnj2ejoc

**hypothesis**: Plain English: the x0.1 reward-scale arm finally got PPO's optimizer unstuck, but only in the last ~15% of its 2M budget -- give the SAME policy 4M more steps to see if walking behavior actually starts moving now that learning is mechanically alive. RETRY of cw-walkcurr-pf-fwd6-rscale10-cont1, which died at launch because respec carried the from-scratch parent's --activation-fn elu into an --init-from warm start (trainer SystemExit); this retry clears the flag (warm start keeps the checkpoint's own ELU) and changes nothing else. Parent cw-walkcurr-pf-fwd6-rscale10 ended with clip_fraction recovered and RISING (0.021@1632k -> 0.079@2016k, never exactly 0 -- unique in the fwd6 wave; gru/sde both collapsed and froze), value_loss 8-18 vs 400-2400 unscaled, approx_kl 0.02, std creeping off init -- but eval behavior still the static crouch (gate 0/6 det, freeprog -0.05..-0.07). ~300k effective post-recovery optimization steps is too few to expect gait discovery; this is the 08-21 continue branch of the parent's own pre-registered mechanism-health gate. If-true: walk_freeprog_score leaves the [-0.10,-0.05] band and trends toward 0, det gate shows nonzero prog. If-false (freeprog flat ANOTHER 4M with clip_fraction staying healthy >0.02): optimizer-crush was necessary but NOT sufficient -- escalate to RND state-novelty or rung-0 sub-goal per STATUS, same-recipe lineage closed. Strongest alternative: sibling rscale50 (x0.02) un-crushes from step 0 and answers faster.

**gate**: Rung-1 gate (same as fwd1-fwd6): C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes, video shows real stepping. Mechanism-health at +4M: freeprog_score > -0.02 or clearly risen out of [-0.10,-0.05] = discovery started (continue/iterate); freeprog still flat in band WITH clip_fraction healthy (>0.02 majority of run) = optimizer-crush necessary-but-insufficient, escalate to RND/rung-0, same-recipe lineage closed.

**verdict**: Fixing the optimizer was necessary but not sufficient at this dose: with reward scaled x0.1 the policy's PPO machinery is finally healthy, yet after 6M total steps the robot still just stands frozen in a splayed crouch and never takes a step. Evidence: train/clip_fraction healthy the entire +4M continuation (0.03-0.16, never the exactly-0 collapse of the 9 frozen rung-1 arms), value_loss ~6 (vs 400-2000 pre-fix), std moving off init (0.372->0.376) -- the crush is gone. But walk_freeprog_score stayed flat in [-0.05,-0.08] the whole run (never trending to 0), det gate 0/6 with speed 0.002 m/s, dir_err 94deg, gait_valid 0/6 (leg 2 sacrificed), det video = identical static crouch in all 10 frames; session eval shows v=(0.000,0.000) under every walk command. Aligned reward+eval (both flat/bad), adequate budget -> genuine FAIL per its own pre-registered branch: optimizer-crush is necessary-but-insufficient at the x0.1 dose, rscale10 same-recipe lineage CLOSED. Next: the escalation fork (RND state-novelty / rung-0 sub-goal per track STATUS) is decided jointly with sibling cw-walkcurr-pf-fwd6-rscale50-cont1 (x0.02 dose, just finished, owned by its own triage cycle) -- if that dose also plateaus below 0, fire escalation (a)/(b); if it crossed 0, the lever is dose-sensitive and rscale50 continues.

