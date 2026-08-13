# cw-arch-noslipphase1-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-13T02:11:34+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-arch-noslipphase1

**wandb_id**: m95w08mf

**hardware_ready**: False

**hypothesis**: One-shot retry of cw-arch-noslipphase1 with the measured fail mode fixed at the INIT, nothing else changed. Parent proved the phase clock fixes expressibility (BC init walks closed-loop: probe 951, progress 0.73) yet fine-tuning still destroyed it. Measured root cause: the init shipped log_std -1.0 (sigma 0.37) and the quasi-static gait dies under that sampling noise - deterministic 1027 vs stochastic 79 - so PPO never samples the taught behavior; and the BC critic was random, so early updates ran on garbage advantages. This init bakes log_std -3.0 (sigma 0.05, matching the DART demo noise the clone was trained to recover from; stochastic return 770 = 75% of deterministic, i.e. on-policy data now LIVES in the no-slip basin) and a value head pretrained on MC returns from 24 deterministic rollouts (rmse ~53 on ~250-scale returns). With income, optimizer, expressibility, exploration noise, and critic all aligned, PPO should finally preserve and polish the first RL no-slip walker.

**gate**: Band-matched phase probe (vref1 base, band 0.008-0.016, park_duty 0, walk_phase_obs 1 @ hz 0.1666667, CMD 0.012, 3 seeds x fwd/crab): TOTAL_RETURN >= 900 (init det 1030) AND progress_ratio >= 0.6 AND walk_loadslip_factor >= 0.5 AND no crouch (height factor >= 0.75) - the fine-tune must NOT erode the working init.

**verdict**: FAIL on the pre-registered gate, but the good branch of the two predicted outcomes: the init fix (log_std -3, MC-pretrained critic) CURED the violent destruction mode from the parent (cw-arch-noslipphase1) — zero pitch-rocking, roll_tail 0.3-2.3deg both modes, gait_valid 6/6 det+sto with ZERO sacrificed legs and balanced duty (0.36-0.76 across all six legs both modes), a genuine honest six-leg walk, video-confirmed both fresh eval passes (gate DR0 + own-DR0.35). What replaces it is a slower basin-drift failure, exactly as this cycle's own follow-up (r2) already diagnosed: over the 2M steps env/walk_loadslip_factor collapsed 1.00 (step 131k) -> 0.069 (final) while reward climbed (quarters 64/171/304/369) and env/walk_height_factor never cleared the 0.75 no-crouch bar (peaked 0.737) — 2/6 det episodes end with real 44-66mm height sag (video-confirmed sinking, not a labeling artifact). PPO is trading the taught no-slip foot-anchoring for forward-progress income under the MJX training env's pricing. TOTAL_RETURN, walk_loadslip_factor>=0.5, and height-factor>=0.75 gate clauses all miss.

