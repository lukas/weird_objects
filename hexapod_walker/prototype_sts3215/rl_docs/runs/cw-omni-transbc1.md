# cw-omni-transbc1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-11T10:04:59+00:00

**pod**: hexapod-mjx-train-9

**steps**: 2000000

**parent**: cw-omni-trans1

**wandb_id**: 2e9q2t4g

**hardware_ready**: False

**hypothesis**: Teach the robot to walk in ANY commanded direction (backward and sideways included) by giving the trainer a correct example step to imitate at every commanded tick, alongside the unchanged reward; this arm tests whether that imitation anchor prevents the paddling/leg-freezing collapse that killed all three previous any-direction arms. Why the anchor and not another reward change: the 08-11 term-by-term income audit (probe_walk_income, logs/probe_walk_income/) shows this exact stack already pays honest gait 2-4x above every observed degenerate pattern in all four directions at DR 0 AND 0.5, and the collapsed trans1 checkpoint earns BELOW a freeze (205 vs 217) under its own reward - the collapse is missing gradient structure (nothing tells a churning leg which WAY to move), the same signature the BC anchor already fixed twice (rise cw-stand-bc1, hold cw-stand-holdbc1). The imitation target is the command-conditioned scripted TripodGait - the open-loop gait that walks/crabs the REAL robot - emitted one tick ahead on commanded walk ticks only (stop ticks unsupervised). ONE variable vs cw-omni-trans1: train.bc_anchor_coef=1.0 (plus the walk bc_target emission code, snapshot HEAD). Prediction-if-true: at 2M, six-foot stepping in all four eval directions with no pinned leg pair, bc_anchor_loss falling like the rise smoke (0.198->0.04), train/std flat. Prediction-if-false: the paddle/sacrifice fingerprint reappears WITH the anchor engaged and its loss low - imitation anchoring is insufficient for direction generalization; next lever is rot-60 exact equivariance (trans1's pre-registered alternate), not more reward work. Strongest alternative: the anchor works but caps gait quality at scripted level (progress_ratio ~0.35-0.38) - acceptable for discovery, judged at hardening.

**gate**: At 2M (discovery, binary): det eval video shows all six feet cycling contact/swing under forward, backward, lateral and diagonal commands (no leg pair with duty >0.9 - the trans1 paddle fingerprint); det progress_ratio >= 0.25 in EVERY direction (scripted-anchor band 0.35-0.38 under this stack); zero falls; train/bc_anchor_loss decreasing; train/std not past 2x start. KILL early on the trans1 signature: std past 2x start with progress_ratio < 0.2, or anchor loss flat at init scale (silent no-op - the pool-restore lesson).

**verdict**: FAIL -- known WALK exploit persists despite the anchor converging: march-in-place/high-slip paddling with near-zero net displacement (median fwd 0.01m/episode, prog_ratio med 0.09 det / 0.05 sto vs gate>=0.25; slip/m 6-19 vs champion ~1.2-1.5; own-DR pass sacrifices 3 legs in one det episode). Video confirms the floor checker pattern never shifts under the body in any of 12 det+sto clips. bc_anchor_loss DID converge (0.14->0.0097, lower than the rise/hold precedent 0.04) and train/std stayed flat (0.37->0.40, no blowup) -- this is the pre-registered prediction-if-false: imitation anchoring alone is insufficient to prevent the direction-generalization collapse. 4th distinct/near-distinct omni-translation failure (mirror1-r1 freeze, mirror2 leg-sacrifice, trans1 paddle-stall, now transbc1 paddle-with-anchor). Per RESEARCH_RULES two-miss rule: do not requeue another BC-anchor dose/reward variant on this stack -- next lever is the pre-registered alternate, rot-60 exact equivariance.

