# cw-arch-noslipphase1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-13T01:47:52+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-dep-bcnoslip2

**wandb_id**: urcs62xp

**hypothesis**: Representational fix for the no-slip BC line (cross-track: hw dep lineage escalated to arch after reward and optimizer levers both closed). cw-dep-bcnoslip2 proved the income landscape is right (clamp-fit teacher out-earns drag 1022 vs 747 in the matched band) but the clock-driven gait is not expressible reactively: the 72-obs BC init stalled closed-loop at progress 0.03 and PPO turned the stall into pitch-rocking (-481). This run turns on the EXISTING walk phase clock obs (goal.walk_phase_obs=1, +2 dims sin/cos, goal.walk_phase_hz=0.1666667 = one revolution per 6s clamp-fit gait cycle) - used here as an OBSERVABILITY feature, not the refuted k_phase_contact reward prior (that stays 0). BC collection derives the teacher clock by unwrapping the phase obs, so env clock and teacher phase align exactly including settle/park prefixes. Result: the phase-aware BC init WALKS closed-loop before any RL - band-matched probe return 951 vs teacher 1022 (progress 0.73 vs 0.03 without the clock, anchor 1.00, loadslip 0.53, pitch charge zero). RL fine-tune (gentle lr 1e-4 / kl 0.01) now starts from a working no-slip walker in an income landscape whose paid optimum IS that gait; expected outcome is preserve + polish (loadslip and prog factors up), the first RL no-slip walking model.

**gate**: Band-matched phase probe (vref1 base, band 0.008-0.016, park_duty 0, walk_phase_obs 1 @ hz 0.1666667, CMD 0.012, 3 seeds x fwd/crab): TOTAL_RETURN >= 900 (init 951, teacher 1022) AND progress_ratio >= 0.6 AND walk_loadslip_factor >= 0.5 AND no crouch (height factor >= 0.75) - i.e. the fine-tune must NOT erode the working init.

