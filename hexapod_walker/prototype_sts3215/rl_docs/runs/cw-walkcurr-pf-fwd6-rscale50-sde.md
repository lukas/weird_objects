# cw-walkcurr-pf-fwd6-rscale50-sde

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-24T02:46:59+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-rscale50

**hypothesis**: Plain English: dose-decoupled retest of gSDE, same reasoning as the rscale50-gru sibling -- the original fwd6-sde arm was trained at the raw unscaled v2e dose (term_penalty=1200) and its own W&B history shows the identical clip_fraction-collapses-to-0-and-stays-there crush signature every unscaled MLP arm shows, so temporally-correlated action noise never actually got a healthy optimizer to integrate into real displacement. Single lever vs fwd6-rscale50: --use-sde (gSDE, one noise draw per rollout instead of i.i.d. per-tick) on the exact x0.02-scaled crush-fixed recipe, fresh 2M discovery, no warm start. Prediction-if-true: with the value gradient no longer crushed, a single per-rollout noise draw biases the action mean in a fixed direction long enough to integrate into a real forward excursion PPO can reinforce -- env/walk_freeprog_score crosses zero and/or det gate shows real stepping. Prediction-if-false (identical belly-sit signature, healthy clip_fraction): noise STRUCTURE is exonerated even with a healthy optimizer, same clean-isolation logic as the gru sibling -- read jointly, if BOTH rscale50-gru and rscale50-sde reproduce the identical height_err_end_mm~116mm belly-sit signature, every architecture/exploration-structure lever this track has is exhausted and the track's own pinned fork (BC-kickstart values question, OPERATOR_QUESTIONS.md q_20260824T0233Z) becomes the honest next decision, not a further variant.

**gate**: Same rung-1 gate as every fwd6 arm: C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes. Mechanism-health: clip_fraction stays healthy; env/walk_freeprog_score trend vs the [-0.10,-0.05] band.

**refused_reason**: hexapod-mjx-train-4 already runs cw-walkcurr-pf-fwd6-actbias1-idleterm1 — GPU pods host exactly one run; pick a free GPU pod.

