# cw-walkcurr-pf-fwd6-idleterm2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-24T02:25:19+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-rscale50

**wandb_id**: hq4ov3ds

**hypothesis**: Plain English: dose-timing sibling of idleterm1 -- same qvel-frozen-policy discriminator and dedicated penalty (3.0, x0.02-scaled), but HALF the grace+eviction window (1.5s+1.5s=3s total vs idleterm1's 3s+3s=6s), so a frozen episode gets cut roughly twice as fast and PPO sees roughly 2x more distinct episode attempts per wall-clock budget. Tests whether faster eviction (more attempts, less each) beats slower eviction (each attempt gets more settle time before being judged) for actually breaking the static-crouch optimum, or whether 1.5s is too short to distinguish real post-reset settling jitter from genuine freezing (a false-positive risk idleterm1's longer window avoids). Same recipe otherwise (fwd6-rscale50, x0.02 stack, fresh 2M discovery, no warm start). Read jointly with idleterm1: if BOTH show the same identical belly-sit/frozen-crouch signature, idle-termination is refuted regardless of timing dose and BC-kickstart is next; if idleterm2 uniquely breaks through (or uniquely fails via false-positive early terminations dominating a healthy exploring policy), the timing window itself is the lever to tune further.

**gate**: Same rung-1 gate as idleterm1: C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes. Mechanism-health check: env/walk_freeprog_score trend vs the [-0.10,-0.05] band; mean episode length should not sit flat at ~3s (this arm's grace+idle_terminate_s) for the whole run; clip_fraction stays healthy.

**verdict**: Result: timing-dose sibling of cw-walkcurr-pf-fwd6-idleterm1 (half the grace+eviction window, 1.5s+1.5s vs 3s+3s) -- reproduces the identical belly-sit signature, closing the timing-dose axis as a non-factor. Evidence: DR-0 det gate 6/6 episodes terminate 'walk_idle_terminate', prog med 0.02, fwd 0.01m/25s, slip/m 5.60; eval/walk/height_err_end_mm=116.3182mm -- matches idleterm1's 116.3183mm to 4 significant figures. gait_valid reads 6/6 True (no sacrificed legs) unlike idleterm1's 0/6 -- checked on video (walk_det_0, 6-frame strip), NOT a qualitatively different (real-stepping) outcome: the same active splay-and-sink collapse (h_err +20mm -> -170mm over ~2s) as idleterm1, just passing through a transiently-balanced duty spread on its way to the identical static collapse pose before eviction -- gait_valid's sacrificed-leg check is a poor discriminator on a still-collapsing (not yet settled) trajectory. W&B (hq4ov3ds) mirrors idleterm1 point-for-point: env/walk_freeprog_score -0.0998->-0.0215 (same monotonic-but-never-crossing-zero trend), rollout/ep_len_mean 16->491 (same noise-defeats-the-detector shape, see idleterm1's verdict for the mechanism), train/clip_fraction 0.013->0.079 (healthy, no crush). Aligned FAIL per 08-21. CONCLUSION (joint with idleterm1): the qvel-frozen-policy idle-termination mechanism is refuted at both a loose (6s total) and tight (3s total) grace+window dose -- timing is not the missing lever, the belly-sit basin itself is not escaped by adding this termination at the tested penalty (3.0, x0.02-scaled). This closes the 10th independently-refuted rung-1 mechanism class (after RND x1-100 dose+budget, rung-0 swing-income x2, --gru, --use-sde, reward-scale dose+continuations, height-gate loose/tight, park_duty confound+dose+actbias). Per the track's own pinned fork, BC-kickstart is now the only unexplored escalation (flagged to the operator already in OPERATOR_QUESTIONS.md). Evidence: logs/ckpt_eval/cw_walkcurr_pf_fwd6_idleterm2_gate/, W&B hq4ov3ds.

