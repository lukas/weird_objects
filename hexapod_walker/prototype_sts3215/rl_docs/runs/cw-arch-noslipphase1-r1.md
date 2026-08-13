# cw-arch-noslipphase1-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-13T02:17:03+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-arch-noslipphase1

**hypothesis**: One-shot retry of cw-arch-noslipphase1 with the measured fail mode fixed at the INIT, nothing else changed. Parent proved the phase clock fixes expressibility (BC init walks closed-loop: probe 951, progress 0.73) yet fine-tuning still destroyed it. Measured root cause: the init shipped log_std -1.0 (sigma 0.37) and the quasi-static gait dies under that sampling noise - deterministic 1027 vs stochastic 79 - so PPO never samples the taught behavior; and the BC critic was random, so early updates ran on garbage advantages. This init bakes log_std -3.0 (sigma 0.05, matching the DART demo noise the clone was trained to recover from; stochastic return 770 = 75% of deterministic, i.e. on-policy data now LIVES in the no-slip basin) and a value head pretrained on MC returns from 24 deterministic rollouts (rmse ~53 on ~250-scale returns). With income, optimizer, expressibility, exploration noise, and critic all aligned, PPO should finally preserve and polish the first RL no-slip walker.

**gate**: Band-matched phase probe (vref1 base, band 0.008-0.016, park_duty 0, walk_phase_obs 1 @ hz 0.1666667, CMD 0.012, 3 seeds x fwd/crab): TOTAL_RETURN >= 900 (init det 1030) AND progress_ratio >= 0.6 AND walk_loadslip_factor >= 0.5 AND no crouch (height factor >= 0.75) - the fine-tune must NOT erode the working init.

**refused_reason**: hexapod-mjx-train-0 already runs cw-arch-noslipphase1-r1 — GPU pods host exactly one run; pick a free GPU pod.

