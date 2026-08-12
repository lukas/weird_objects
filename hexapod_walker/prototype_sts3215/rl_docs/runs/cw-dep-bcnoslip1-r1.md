# cw-dep-bcnoslip1-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-12T15:01:37+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-dep-bcnoslip1

**wandb_id**: vg2bzthw

**hypothesis**: One-shot retry of cw-dep-bcnoslip1 per the bcgait1 recipe fail-mode-(a) prescription (revert-to-crouch = walk income overwhelms the init): same no-slip BC init and band-matched stack (0.02-0.04, park-duty off), but gentler optimization - lr 3e-4 -> 1e-4 and target-kl 0.02 -> 0.01 - so the fine-tune cannot leave the teacher basin in 2M steps. The parent run erased a healthy init (probe return 562 -> -67, tall +15mm -> crouch -50mm) while the drag-teacher twin bcgait1 survived the SAME optimizer settings, so the no-slip basin is measurably shallower; if true: the r1 ckpt stays tall (probe_tall_wall >= -20mm) and holds loadslip >= 0.15 with progress >= init 0.48. If false (still craters even at lr 1e-4/kl 0.01): the basin is too shallow for PPO income fine-tuning at ANY safe step size under this reward - the remaining levers are reward-side (slip-gated income) or env-side (servo profile), not optimizer-side.

**gate**: Same as parent (pre-registered): probe_walk_income on vref1 base + band 0.02/0.04 + k_park_duty=0 at CMD 0.03, forward+crab x seeds 0,1,2: TOTAL_RETURN > 588 AND walk_loadslip_factor >= 0.15 AND progress_ratio >= 0.55, 0 terminations; secondary probe_tall_wall steady height >= -20mm.

**verdict**: FAIL, same fail mode (a) as the parent - the optimizer-side lever is now CLOSED. lr 1e-4 + target-kl 0.01 (the bcgait1-recipe one-shot retry) still erased the healthy no-slip BC init in 2M steps: band-matched probe TOTAL_RETURN 6 vs teacher 593, progress_ratio 0.10, walk_loadslip_factor 0.06 (drag level), probe_tall_wall -47 mm crouch / 262 mm splay / pitch margin pinned. Paired with the parent (lr 3e-4/kl 0.02 -> return -67), this brackets PPO income fine-tuning: under this reward stack (even with the speed band lowered to the teacher envelope 0.02-0.04 and park-duty off) the crouch-splay attractor out-earns the no-slip gait basin at every safe step size, while the SAME optimizer preserved the drag-tripod teacher (cw-dep-bcgait1 PASS). Conclusion: training a no-slip gait needs reward-side change (slip-gated income, e.g. multiply walk kernel by loadslip/anchor factors) or env-side change (servo profile that makes clean swings executable - the fitted 31 deg/s clamp makes the no-slip teacher itself drag in the training env). Launch-mechanics note: this run was created by the first respec attempt whose snapshot push flaked; a recovery path started it ~10 min later - the training itself is clean (verified extra_args).

