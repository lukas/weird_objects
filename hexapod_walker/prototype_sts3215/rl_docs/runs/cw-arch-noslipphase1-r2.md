# cw-arch-noslipphase1-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-13T02:21:41+00:00

**pod**: hexapod-mjx-train-0

**steps**: 1000000

**parent**: cw-arch-noslipphase1-r1

**wandb_id**: nh8kthf3

**hypothesis**: Drift-budget arm of the phase-clock no-slip line. r1 fixed expressibility (phase obs), exploration noise (log_std -3 baked in, unchanged after 2M steps - verified) and the critic (MC-pretrained), and the destruction mode is GONE: r1 walks stably (progress 0.64, pitch charge -1.6, hold err 0.8-2.0 deg all run). What remains is slow basin drift: over 2M steps the policy traded foot anchoring for progress (loadslip factor 0.53 -> 0.11, C-probe return 951 -> 617) even though the anchored gait earns more in the C probe - evidence the MJX training env prices anchoring weaker than the C env, so PPO leaks toward rolling loaded feet. This arm halves the drift budget on every axis: 1M steps (r1 periodic evals were already healthy at 1M), lr 1e-4 -> 5e-5, target-kl 0.01 -> 0.005. Same init (ppo_goal_cw_bcnoslip_phase2_init, det probe 1030 / stochastic 770). Expected: preserve anchoring (loadslip >= 0.5) while keeping the stability polish; if it STILL erodes, the recorded blocker is the MJX-vs-C anchoring-price gap and the next lever is pricing, not optimization.

**gate**: Band-matched phase probe (vref1 base, band 0.008-0.016, park_duty 0, walk_phase_obs 1 @ hz 0.1666667, CMD 0.012, 3 seeds x fwd/crab): TOTAL_RETURN >= 900 (init det 1030) AND progress_ratio >= 0.6 AND walk_loadslip_factor >= 0.5 AND no crouch (height factor >= 0.75).

**verdict**: INVALID - do not read as evidence. The aborted r1 respec plus two refused launch retries left duplicate r1 ledger entries (one empty, one with only --n-envs); this respec inherited the bare duplicate, so r2 trained FROM SCRATCH on the default task mix with no --init-from, no walk-only goal mix, no band, no phase obs. Ledger deduped (backup /tmp/experiments_backup_*.json); the real drift-budget arm is relaunched as cw-arch-noslipphase1-r3.

