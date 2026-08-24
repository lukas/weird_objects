# cw-walkcurr-pf-fwd6-stagea-slip1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-24T00:06:19+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-rscale50

**wandb_id**: rwdahh6d

**hypothesis**: Stage-A flat-forward slip diagnostic arm from the current fwd6-rscale50 baseline. Keep the same PPO/MJX infrastructure and 18 raw joint-target action representation, but simplify the command diet to exactly 0.05 m/s forward on flat ground, loosen roll/pitch safety so early clumsy exploration is not cut short, and add a modest planted-foot tangent slip charge only on meaningful consecutive contact. The slip term is averaged across measured feet, deadbanded, and capped, so it is a cleanup pressure and diagnostic, not the main objective. If true: optimizer health from rscale50 stays healthy, per-foot tangent-contact velocity and swing/contact metrics appear in W&B, reward_foot_slip_tangent remains small relative to existing loadslip/idle charges, and walk_freeprog_score/swing-contact telemetry leave the static-crouch basin earlier than the parent. If false: reward_foot_slip_tangent dominates or the same static crouch persists with near-zero liftoff/touchdown counts, closing this lever without blaming the 18-action interface.

**gate**: Discovery health at 2M vs cw-walkcurr-pf-fwd6-rscale50: trainer launches cleanly from code bad7d510; clip_fraction remains healthy (>0.02 last quarter) and std moves off init; W&B includes reward_foot_slip_tangent, walk_tangent_contact_vel_mean/max, walk_contact_meaningful_feet, per-foot tangent/contact/swing keys; reward_foot_slip_tangent stays modest (not the dominant negative term); fixed-forward C-env gate is unchanged in spirit: zero tilt terms, cmd_prog_frac >=0.35, direction_err <=30 deg, slip/m <=3.0, six legs cycling on >=4/6 det episodes, video shows real stepping. Mechanism read if gate fails: compare liftoff/touchdown/swinging_feet and tangent velocity against parent static-crouch metrics.

