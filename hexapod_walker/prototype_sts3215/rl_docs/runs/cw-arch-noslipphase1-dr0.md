# cw-arch-noslipphase1-dr0

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-13T03:42:57+00:00

**pod**: hexapod-mjx-train-0

**steps**: 1000000

**parent**: cw-arch-noslipphase1-r3

**wandb_id**: km5wor4p

**hardware_ready**: False

**hypothesis**: Test whether the no-slip walking policy stops trading away its foot anchoring when the training world is CLEAN instead of randomized - if it does, the anchoring erosion in r1/r3 was domain randomization making slip invisible to the reward, not the GPU physics paying it. Audit evidence (probe_noslip_income, logs/probe_noslip_income/ on train-0, tag exp/probe-noslip-income-audit): under the training distribution (DR 0.35 + 30% tipped + stochastic) the C env ALSO pays forward drift (r1 510 vs init 470 per-ep) and the anchoring signals go blind (anchor frac 0.95 vs 0.91, loadslip ratio 1.42 vs 1.45 m/m), while at DR-0/det the C env prefers the init by +280 (loadslip 0.54 vs 0.10) - so 'MJX prices anchoring weaker than C' is NOT needed to explain the drift; the training distribution is. This arm = r3's exact recipe/dose (1M steps, lr 5e-5, kl 0.005, phase2 init) with the training distribution cleaned (dr-scale 0, tipped 0). Prediction-if-true (DR-blinding): training walk_loadslip_factor holds >=0.5 through 1M and the band-matched gate passes at a dose where r3 eroded to 0.45. Prediction-if-false (real MJX-vs-C physics gap): loadslip erodes to <=0.45 even at DR 0. Strongest alternative: optimizer-dose artifact - excluded by matching r3's dose exactly.

**gate**: Band-matched phase probe (vref1 base, band 0.008-0.016, park_duty 0, walk_phase_obs 1 @ hz 0.1666667, CMD 0.012, 3 seeds x fwd/crab): TOTAL_RETURN >= 900 AND progress_ratio >= 0.6 AND walk_loadslip_factor >= 0.5 AND height factor >= 0.75. MECHANISM CLAUSE regardless of pass: final training env/walk_loadslip_factor >= 0.5 => DR-blinding CONFIRMED, next lever is a DR curriculum (lock gait clean, anneal DR in); <= 0.45 => MJX-vs-C contact parity audit is the lever, NO more training arms on this line.

**verdict**: FAIL per gate (C-probe return 881<900, loadslip 0.31<0.5; progress 0.73 and height 0.86 pass) - and the pre-registered mechanism clause fires on the FALSE branch, decisively: at DR-SCALE 0 (clean world, tipped 0, r3's exact 1M/5e-5 dose) MJX PPO STILL erodes training walk_loadslip_factor 1.00->0.085 by ~400k steps (anchor frac stays 0.957 = touchdown-reset creep the anchor gate cannot see) while its own return climbs 31->464. The matched C-env income audit (probe_noslip_income, logs/probe_noslip_income/ on train-0) shows the C env CHARGES this same drift ~-280/ep at DR0/det (init 847 vs r1 567 under training pricing) - so DR-blinding is REFUTED as the primary mechanism and the MJX-vs-C anchoring/contact gap is CONFIRMED and quantified: the same dr0 checkpoint measures loadslip 0.085 in MJX vs 0.31 in C. Per the gate's consequence clause: NO more training arms on this line; the lever is a warp-vs-C loaded-foot contact-creep parity audit (replay matched action streams det in both physics, compare per-tick loaded-foot XY displacement; warp iterations-1/4 contact looseness is the suspect). Note: run completed healthy in 198s; the launcher's fast-finish false-FAILED is fixed this cycle (launch_run.py verify). Ckpt ppo_goal_cw_arch_noslipphase1_dr0.zip md5 d1019494. || PARITY AUDIT ADDENDUM (08-13 ~06:xx, probe_contact_parity.py): the 'MJX-vs-C contact gap' read is CORRECTED - matched command-stream replays show warp@1/4 within ~3-6% of C@50 on loaded-foot slip (both 0.055 and 0.012 m/s regimes), flat across solver iterations, zero warp stance creep; the 0.085-vs-0.31 comparison mixed the stochastic on-policy training metric with a det probe (C's own stochastic replays measured ratio 1.42-1.45 = MJX's ~1.44), amplified by the loadslip-factor clip. The det erosion (factor 0.31 at the 1M dose) is real RL drift, not physics. Line concludes at r4; no physics fix pending.

**failed_reason**: W&B global_step not advancing (1048576 -> 0) after 120s (n_steps=64, cpu-time flat for 2 polls)

