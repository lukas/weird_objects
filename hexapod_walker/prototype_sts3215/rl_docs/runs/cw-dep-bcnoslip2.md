# cw-dep-bcnoslip2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-12T18:08:28+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-dep-bcnoslip1-r1

**wandb_id**: c3x0g4mg

**hypothesis**: Third arm of the no-slip BC line, attacking the INCOME side instead of the optimizer: both earlier arms used a teacher whose timing does not fit the training env 31 deg/s servo clamp, so it priced BELOW the crouch attractor and fine-tuning erased it. This run uses the clamp-fit teacher (NoSlipGait.CLAMP_FIT_KW: period 6s, shift 0.10, swing 0.40, lift 20mm, alpha 1 - zero true scrub under the training write profile per contact-Jacobian audit) and matches the band to what that teacher can execute: 0.008-0.016 m/s, park-duty off. Band-matched walk-env probe: the teacher now OUT-EARNS the drag gait 1022 vs 747 (loadslip factor 0.72 vs 0.10, anchor frac 1.00, drag 0) - first configuration where the paid optimum IS the no-slip gait. Init = BC from band-matched demos (holdout action err 0.0154; closed-loop it stalls at progress 0.03 because the clock-driven teacher phase is not fully recoverable from obs 72 - consolidating the cycle is exactly what income now pays for). Optimizer stays at the gentle r1 settings (lr 1e-4, target-kl 0.01).

**gate**: Band-matched probe (vref1 base, band 0.008-0.016, park_duty 0, CMD 0.012, 3 seeds x fwd/crab): TOTAL_RETURN >= 700 (teacher 1022, stand-still init 335) AND progress_ratio >= 0.4 AND walk_loadslip_factor >= 0.5 AND probe_tall_wall >= -20mm (no crouch collapse).

**verdict**: FAIL on the pre-registered gate, NEW fail mode (c) pitch-rocking - the income-side lever WORKED but did not suffice. Band-matched probe (vref1 base, band 0.008-0.016, park_duty 0, CMD 0.012): TOTAL_RETURN -481 vs teacher 1022 and stand-still init 335; reward_pitch -449 dominates, progress 0.12, anchor_frac 0.21, loadslip 0.32. NO crouch this time (height factor 0.85 = teacher; the crouch attractor is priced out, confirming the clamp-fit teacher + matched band fixed the reward landscape). What failed instead: the BC init cannot express the clock-driven gait reactively (closed-loop progress 0.03 at obs 72 - teacher phase not recoverable from a single frame), and 2M gentle PPO steps turned the stall into pitch-rocking rather than discovering the cycle (trainer periodic hold err drifted 9.8 -> 18.6 deg). Reward/optimizer levers are now BOTH closed; the remaining lever is representational: phase/clock feature in obs, recurrence or frame-history for the walk head, or DAgger-style imitation - all architecture-track work, not another respec of this line.

