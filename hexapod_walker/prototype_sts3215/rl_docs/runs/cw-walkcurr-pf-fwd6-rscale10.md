# cw-walkcurr-pf-fwd6-rscale10

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PARTIAL

**created**: 2026-08-23T20:37:41+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd3-chargeramp

**wandb_id**: pvmsnfzl

**hypothesis**: Plain English: the robot never learns to walk because PPO's own optimizer is crushing its learning signal -- the reward numbers are simply too BIG, so this arm shrinks every reward gain 10x without changing what the reward prefers. Dig-in evidence (all 8 FAILed rung-1 arms, cached W&B): train/clip_fraction hits EXACTLY 0 by 10-38% of every run, approx_kl ~3e-5, policy std frozen at its init 0.3685, while train/value_loss sits at 400-2000+ (returns are |1000s|-scale under v2e pricing). SB3 clips the GLOBAL grad norm (max_grad_norm=0.5) over policy+value params in ONE optimizer, so the value head's huge gradients rescale the policy/log_std/entropy gradients toward zero -- explaining why pricing, init-noise (logstd0 collapsed FASTEST), and 10x-entropy (std moved only 0.369->0.376 in 2M) all failed identically. SB3 normalizes advantages per minibatch, so a global gain scale leaves the policy gradient invariant and shrinks ONLY the value gradient: the crush lifts, nothing about the incentive ordering changes. All 15 active gains scale together (7 recipe gains + 8 base compute_reward gains incl. k_track -- the scaled bank's first run measured that scaling only the recipe 7 lets the unscaled reward_task posture kernel pay park +323, destroying the ranking). Bank green at x0.1: test_walkcurr_pf_scaled_* (ranking with scaled margins + behavior-delta linearity), snapshot exp/walkcurr-fwd6-rscale-batch. If-true: clip_fraction stays >~0.02 past 50% of the run, std moves off init, walk_freeprog_score (gain-independent) leaves the flat [-0.10,-0.05] band and trends toward 0. If-false (freeze recurs WITH healthy clip_fraction): optimizer-crush refuted, escalate to RND/rung-0 per STATUS. Strongest alternative: the sibling x0.02 arm if x0.1 still leaves value_loss dominant.

**gate**: Rung-1 gate (same as fwd1-fwd5): C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes, video shows real stepping. Mechanism-health at 2M: clip_fraction still >0.02 in the last quarter AND std off init = the optimizer fix worked mechanically even if behavior is incomplete (continue per 08-21 ruling if freeprog_score also rising); clip_fraction collapsed to 0 again = optimizer-crush hypothesis refuted for this dose.

**verdict**: PARTIAL -- the optimizer-crush mechanism is CONFIRMED at x0.1 reward scale, but behavior is still frozen at 2M. Mechanical result (the gate's own health branch): value_loss drops 305 -> 8-18 (vs 400-2400 unscaled), clip_fraction NEVER hits exactly 0 and RECOVERS to healthy late (0.021@1632k -> 0.062@1824k -> 0.079@2016k, rising), approx_kl 0.02 at end, std creeping off init (0.3678->0.3692) -- the only arm of the fwd6 wave (gru/sde both clip->0, frozen) where PPO is still updating, exactly as the cross-prediction demanded. Behavioral result: gate 0/6 det (prog 0.00, static crouch holding leg 3 up -> gait_valid 0/6 sac=[3]), walk_freeprog_score left the -0.10 start but hovers -0.05..-0.07, not yet rising. Reading: clip_fraction only became healthy in the final ~15% of the run -- the policy got ~300k effective optimization steps, too few to expect discovery. Next: continuation cw-walkcurr-pf-fwd6-rscale10-cont1 (+4M from checkpoint, launched this cycle) -- if freeprog stays flat ANOTHER 4M with clip_fraction healthy throughout, the crush was necessary-but-insufficient and the pre-registered escalation (RND state-novelty / rung-0 sub-goal) fires; sibling rscale50 (x0.02, still training) gives the dose comparison.

