# cw-walk-loadslip

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T11:35:03+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_anchorgate.zip

**wandb_id**: f44g243x

**hardware_ready**: False

**hypothesis**: The ruled structural fix for the cadence-reset exploit (OPERATOR RULING 2026-08-09 WALK-SLIP) and the definitive reward-side slip test. Every anti-slip income lever so far could be evaded by stepping MORE OFTEN because each touchdown reset the anchor allowance (free slip = cadence x tol: c1 +23% stances, tol5 paid an 18% stake, stepdisp12/c36 proved step credit does NOT pay the ride — the allowance-reset protection does). This arm swaps the gating QUANTITY, one variable off champion 35234ddc: walk_anchor_gate 1->0, walk_loadslip_gate=1 — velocity income multiplied by factor of EPISODE-ACCUMULATED loaded slip per meter of along-command progress (ok=0.75, max=1.5, floor 0.05 m; same quantity the harness scores; no touchdown can reset it). Controller scale audit: champion paddle ep keeps ~11% of velocity income, clean ep ~75% — a near-total stake where tol5 was 18%. GPU probe PASS (1M, 0 tb, ep_rew 26->214 climbing). If-true: walk_loadslip_ratio falls / factor climbs EARNED, harness det DR1.0 slip_per_m <=1.0 (champion 1.11 along-command / 1.24 legacy denominator). If-false: factor parks ~0.1-0.3 with income forfeited and slip flat — the policy pays a ~70-90% stake rather than stop sliding, which CLOSES the reward side entirely: slip becomes purely a sim contact-pricing defect (cycle-28 class). Strongest alternative: slip ratio falls via speed/progress collapse, not anchoring — excluded by progress_ratio 0.75-1.25 + speed-band checks. Exploit watch: unloading feet below contact threshold while sweeping (uncounted slip) = duty drop + swing spike; income starvation collapse (terminations, speed under band) = kill per checkup.

**gate**: DR1.0 harness 15s own-cfg 6+6: det slip_per_m <=1.0 AND sto <=1.1, gait_valid 12/12, 0 term, progress_ratio median 0.75-1.25, speed in band; DR0 det retention fwd >=0.55 6/6; frames watched det+sto for anchored stance vs paddle-creep; W&B walk_loadslip_factor trend read against if-false shape (factor parked low, income forfeited)

**verdict**: FAIL / hypothesis REFUTED — pre-registered if-false fired. OBSERVATIONS: W&B f44g243x 20.05M steps; income forfeited during training (env/reward_walk quarters 0.76/0.77/0.63/0.52, walk_prog 0.38->0.216) with walk_speed flat ~0.052 — policy pays the near-total loadslip stake rather than anchor. Harness own-cfg 15s: DR0 det 3/6, slip/m mean 1.54 (all 6 eps 1.31-1.81); DR1.0 (gated condition) det 2/6 slip/m 2.39 vs champion 1.240 same condition — WORSE than parent; sto 2/6, one draw prog -0.21 slip 24.6. gait_valid 24/24, 0 terminations. Frames (det+sto watched, DR0+DR1.0): same paddle-creep as champion — six legs cycling, feet sweeping while loaded, no flag leg, no falls. INTERPRETATION: the episode-accumulated, touchdown-proof slip gate cannot be evaded by cadence, and the policy STILL chose sliding + forfeited income; the gate also degraded slip vs parent. With anchor-gate (moved slip once then price-ceiling), tol5 stake (paid 18%), stepdisp (refuted), and now a ~50-90% episode-level stake all exhausted, no reward-side lever outbids in-sim-free sliding. VERDICT: reward side of skating CLOSED pending seed-1 twin concordance (ruling-7); slip root = sim contact/current pricing (cycle-28 class, operator hardware calibration). NOT hardware-ready. HYPOTHESIS STATUS: refuted exactly along the pre-registered if-false shape.

