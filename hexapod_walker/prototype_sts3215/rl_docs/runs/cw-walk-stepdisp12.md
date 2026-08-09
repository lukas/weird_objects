# cw-walk-stepdisp12

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T10:29:26+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_anchorgate.zip

**wandb_id**: xj6ehawo

**hypothesis**: Cadence-attribution arm (0-c.2 displacement-gated step-event credit; the pre-registered escalation from the CLOSED tolerance rung, cycle 34). Cadence inflation is the walk lineage's recurring adaptation (c1 +23%, tol5 +13% more, fresh init 2x) and two income channels could pay for it: (A) step-event credit collected per touchdown, (B) the extra touchdowns protecting the anchor-gate a_factor (each reset re-grants the slip allowance). This arm removes channel A BY CONSTRUCTION: each paid step credit consumes 12mm of banked net body along-command displacement; marginal touchdowns at fixed distance earn 0 (walk_step_denied logs the denied credit). Config = anchorgate-c1 EXACT (champion 35234ddc warm start, tol=10, seed 0) + reward.step_disp_budget_mm=12 - c1 is the identical-config control that drifted 47->58 stances/ep with step income +24%. Prediction-if-true (A pays the ride): det DR1.0 stance count stays <=~55/ep, walk_step_denied transient then ~0, slip flat ~1.24 (expected, NOT gated), DR0 retention holds. Prediction-if-false (B pays it): cadence drifts to >=~58 anyway with walk_step_denied climbing => both income channels mapped and exhausted => walk slip line NEEDS OPERATOR whole (cycle 28 pricing ruling), no further walk reward arms. Strongest alternative: c1's drift was stochastic noise - excluded by same seed + identical config, one variable. SCALE AUDIT (cycle 34, recorded): this stake is 1-2% of step income at current cadences - VACUOUS as a slip arm; slip is a secondary observation only. Champion cannot be displaced by this run.

**gate**: DR1.0 harness 15s own-cfg 6+6: det stance count <=~55/ep AND walk_step_denied ~0 at end of training, gv 12/12, 0 term; DR0 harness 15s: det fwd mean >=0.55, gv 12/12, fwd-hemi sto fwd>=0.40 5/5 (backward draw recorded-excluded); slip det/sto recorded vs champion 1.240/1.245 as observation (expected flat); frames watched det+sto

**verdict**: FAIL / hypothesis REFUTED. OBSERVATIONS: W&B xj6ehawo finished 20.05M steps; ep_rew quarters [769,875,869,874]. Training: walk_step_denied CLIMBED 0.0031->0.0078/tick (never ~0), walk_step_bank_m drained 0.100->0.059, anchor_frac 0.86->0.90, reward_step_event 0.149->0.180. Harness (own cfg, 15s, seed 0, 6+6 det/sto x DR0+DR1.0): DR1.0 det swings/ep 61.5 [62,63,65,62,52,65] vs gate <=~55 (champion anchorgate 47.2, c1 control 58.2); det slip/m 1.701 (median ~1.48) vs champion 1.240; sto ep1 REGRESSION fwd 0.049m slip 24.4/m (same-seed champion did 0.657 on that draw), sto ep0 0.378<0.40 -> fwd-hemi 4-of-6 vs champion 6/6. DR0 retention holds: det fwd 0.680>=0.55, gv 24/24 all modes, 0 terminations, no sacrificed legs. Frames watched (dr10 det0 md5 2abc8912 375f; dr10 sto1 md5 e9403875 375f; dr10 sto5; dr0 det0 md5 5078247a): sprawled low stance, legs cycling in place with tiny displacements, feet skating - paddle-creep, NOT stepping; sto1 shows stiff extended front legs and zero body translation. Exploit checked and not found: policy did NOT suppress touchdowns to dodge denial (cadence rose). INTERPRETATION: with the step-credit channel closed by construction (denials real and rising), cadence STILL inflated past the c1 control - channel A (step credit) does NOT pay for cadence inflation; the allowance-reset protection (channel B) or no income lever at all governs it. Income-side walk levers now fully mapped and exhausted per pre-registration. VERDICT: FAIL. hardware-ready: NO - paddling transport, feet skate in place; would grind servos on the real robot. HYPOTHESIS STATUS: REFUTED (if-false branch confirmed). Champion unchanged (anchorgate 35234ddc). Next per pre-registration: rulings (5)+(6) arms - already in flight via concurrent cycle (cw-walk-longdist/forwardband).

