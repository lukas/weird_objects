# cw-arch-noslipphase1-r3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-13T02:35:04+00:00

**pod**: hexapod-mjx-train-0

**steps**: 1000000

**parent**: cw-arch-noslipphase1-r1

**wandb_id**: i9y2svov

**hypothesis**: Drift-budget arm of the phase-clock no-slip line (r2 was INVALID - corrupt ledger duplicate gave it no init and the default task; deduped and relaunched as r3). r1 fixed expressibility (phase obs), exploration noise (log_std -3 baked, unchanged after 2M steps) and the critic (MC-pretrained), and the destruction mode is GONE: r1 walks stably (progress 0.64, pitch charge -1.6, hold err 0.8-2.0 deg all run). What remains is slow basin drift: over 2M steps the policy traded foot anchoring for progress (loadslip factor 0.53 -> 0.11, C-probe return 951 -> 617) even though the anchored gait earns more in the C probe - evidence the MJX training env prices anchoring weaker than the C env, so PPO leaks toward rolling loaded feet. This arm halves the drift budget on every axis: 1M steps (r1 periodic evals already healthy at 1M), lr 1e-4 -> 5e-5, target-kl 0.01 -> 0.005. Same init (ppo_goal_cw_bcnoslip_phase2_init, det probe 1030 / stochastic 770). Expected: preserve anchoring (loadslip >= 0.5) while keeping the stability polish; if it STILL erodes, the recorded blocker is the MJX-vs-C anchoring-price gap and the next lever is pricing, not optimization.

**gate**: Band-matched phase probe (vref1 base, band 0.008-0.016, park_duty 0, walk_phase_obs 1 @ hz 0.1666667, CMD 0.012, 3 seeds x fwd/crab): TOTAL_RETURN >= 900 (init det 1030) AND progress_ratio >= 0.6 AND walk_loadslip_factor >= 0.5 AND no crouch (height factor >= 0.75).

**verdict**: NEAR-MISS, dose-response point confirmed: halving the drift budget (1M steps, lr 5e-5, kl 0.005) recovered most of the anchoring r1 lost - loadslip 0.11 -> 0.45 (gate 0.5), return 617 -> 889 (gate 900), progress 0.70, anchor frac 1.00, no crouch. Missed the gate by 11 return points / 0.05 loadslip. Confirms drift-dose is the operative variable; quartered again in r4.

**failed_reason**: W&B global_step not advancing (1048576 -> 0) after 120s (n_steps=64, cpu-time flat for 2 polls)

