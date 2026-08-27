# cw-walkcurr-pf-fwd6-rnd100

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-23T22:44:41+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-rnd10

**hypothesis**: Plain English: rnd02 (coef 0.02) and rnd10 (coef 0.10, 5x) BOTH froze identically (same static splayed crouch, gate 0/6 det, gait_valid 0/6, walk_speed DECAYING 0.10->0.006 m/s over training, direction_err flat ~90deg/chance, rnd/intrinsic_mean decaying near-identically 0.037->0.0055 regardless of coef) -- a 5x dose bracket made no qualitative difference, so this is a 10x-beyond-rnd10 (50x rnd02) bracket to close the dose axis before invoking the track's rule-brushing BC-kickstart last resort. Single lever vs rnd10: --rnd-coef 1.0 (raw per-step reward_rnd_intrinsic already ~0.06 at coef 0.1, comparable to the recipe's other small charges -- at coef 1.0 it becomes the dominant per-step term, testing whether RND needs to actually dominate the crouch's charge-avoidance income to disturb it, or whether dominating it instead triggers pure novelty-chasing/flailing (predicted-if-overshoot: ep_rew_mean task components regress, video shows chaotic non-gaited motion instead of the frozen crouch or instead of rhythmic stepping).

**gate**: Rung-1 gate (same as every fwd6/rnd arm): C-env det fixed-forward panel -- prog_ratio>0 and gait_valid>=4/6 det with visible forward travel on video, env/walk_freeprog_score crosses 0, clip_fraction stays healthy. PASS = rung-1 lands. FAIL (still frozen OR pure flailing with no task progress) closes RND-as-a-class at all three tested doses (0.02/0.10/1.0) -- escalate to BC-kickstart (track item d) as the only unexplored ladder item.

**refused_reason**: a process for cw-walkcurr-pf-fwd6-rnd100 already exists on hexapod-mjx-train-1

