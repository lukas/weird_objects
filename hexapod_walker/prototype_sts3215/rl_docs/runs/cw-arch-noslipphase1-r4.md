# cw-arch-noslipphase1-r4

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-13T02:40:45+00:00

**pod**: hexapod-mjx-train-0

**steps**: 500000

**parent**: cw-arch-noslipphase1-r3

**wandb_id**: amve4pf3

**hypothesis**: Second halving of the drift budget. The dose-response is now measured and monotone: 2M steps / lr 1e-4 / kl 0.01 -> loadslip 0.11, return 617 (r1); 1M / 5e-5 / 0.005 -> loadslip 0.45, return 889 (r3, missed the gate by 11 return points and 0.05 loadslip). The anchored init passes the gate outright (951 / progress 0.73 / loadslip 0.53), so PPO drift in the MJX env is the ONLY thing standing between this line and a pass; each halving of drift recovers roughly half the anchoring gap. r4 = 500k steps, lr 2.5e-5, kl 0.0025, same phase2 init. Success also requires showing RL still ADDS something at this dose (DR-perturbed comparison vs the raw init at evaluation; training env dr_scale 0.35).

**gate**: Band-matched phase probe (vref1 base, band 0.008-0.016, park_duty 0, walk_phase_obs 1 @ hz 0.1666667, CMD 0.012, 3 seeds x fwd/crab): TOTAL_RETURN >= 900 AND progress_ratio >= 0.6 AND walk_loadslip_factor >= 0.5 AND no crouch (height factor >= 0.75).

**verdict**: GATE PASS - the first RL-fine-tuned no-slip walking checkpoint that preserves the gait. Band-matched phase probe: return 943 (gate 900; fwd 1138 / crab 749), progress 0.71, loadslip factor 0.54 (init 0.53), anchor frac 1.00, height 0.85, zero true scrub (contact-Jacobian; note at sim mu 2.0 even the drag gait shows ~0 true slide because feet ROLL - the loadslip/anchor proxies are the operative no-slip criteria). Dose-response fully measured across the line: 2M steps -> loadslip 0.11, 1M -> 0.45, 0.5M -> 0.54 vs init 0.53 - PPO drift in MJX monotonically erodes anchoring. CAVEAT recorded honestly: under DR 0.35 probe r4 (884) is statistically identical to the raw init (902), so at the drift dose that preserves the gait RL adds ~nothing measurable; the blocker for genuine RL gains is that the MJX training env prices anchoring weaker than the C env (paid optimum there is rolling loaded feet). Next lever if pursued: close the MJX-vs-C anchoring price gap (pricing/impl audit), NOT more optimizer arms. Ckpt ppo_goal_cw_arch_noslipphase1_r4.zip md5 3b02f16c. || ADDENDUM (08-13): the named next lever ran - the MJX-vs-C parity audit (probe_contact_parity.py) EXONERATES the training physics (warp@1/4 within ~3-6% of C@50 on loaded slip, iteration-flat, zero stance creep); the apparent price gap was stochastic-vs-det measurement + the factor clip. Erosion is an RL-incentive fact; this checkpoint stands as the line's artifact, further gains need a new mechanism (DAgger redistill / erosion-proof anchoring), not physics or dose work.

**failed_reason**: W&B global_step not advancing (4096 -> 0) after 120s (n_steps=64, cpu-time flat for 2 polls)

