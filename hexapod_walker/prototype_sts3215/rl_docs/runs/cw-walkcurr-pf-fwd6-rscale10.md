# cw-walkcurr-pf-fwd6-rscale10

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T20:37:41+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd3-chargeramp

**wandb_id**: pvmsnfzl

**hypothesis**: Plain English: the robot never learns to walk because PPO's own optimizer is crushing its learning signal -- the reward numbers are simply too BIG, so this arm shrinks every reward gain 10x without changing what the reward prefers. Dig-in evidence (all 8 FAILed rung-1 arms, cached W&B): train/clip_fraction hits EXACTLY 0 by 10-38% of every run, approx_kl ~3e-5, policy std frozen at its init 0.3685, while train/value_loss sits at 400-2000+ (returns are |1000s|-scale under v2e pricing). SB3 clips the GLOBAL grad norm (max_grad_norm=0.5) over policy+value params in ONE optimizer, so the value head's huge gradients rescale the policy/log_std/entropy gradients toward zero -- explaining why pricing, init-noise (logstd0 collapsed FASTEST), and 10x-entropy (std moved only 0.369->0.376 in 2M) all failed identically. SB3 normalizes advantages per minibatch, so a global gain scale leaves the policy gradient invariant and shrinks ONLY the value gradient: the crush lifts, nothing about the incentive ordering changes. All 15 active gains scale together (7 recipe gains + 8 base compute_reward gains incl. k_track -- the scaled bank's first run measured that scaling only the recipe 7 lets the unscaled reward_task posture kernel pay park +323, destroying the ranking). Bank green at x0.1: test_walkcurr_pf_scaled_* (ranking with scaled margins + behavior-delta linearity), snapshot exp/walkcurr-fwd6-rscale-batch. If-true: clip_fraction stays >~0.02 past 50% of the run, std moves off init, walk_freeprog_score (gain-independent) leaves the flat [-0.10,-0.05] band and trends toward 0. If-false (freeze recurs WITH healthy clip_fraction): optimizer-crush refuted, escalate to RND/rung-0 per STATUS. Strongest alternative: the sibling x0.02 arm if x0.1 still leaves value_loss dominant.

**gate**: Rung-1 gate (same as fwd1-fwd5): C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes, video shows real stepping. Mechanism-health at 2M: clip_fraction still >0.02 in the last quarter AND std off init = the optimizer fix worked mechanically even if behavior is incomplete (continue per 08-21 ruling if freeprog_score also rising); clip_fraction collapsed to 0 again = optimizer-crush hypothesis refuted for this dose.

