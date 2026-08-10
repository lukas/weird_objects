# cw-uni-rfix-warm1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T13:51:24+00:00

**pod**: hexapod-mjx-train-0

**steps**: 18000000

**parent**: cw-uni-mix0-r1

**wandb_id**: ln94x5rr

**hardware_ready**: False

**hypothesis**: Reward-pricing arm of the c69/c71 uni-line DIG-IN (the rise/lower income audit pre-registered in blend1-r2's if-false). Root-cause audit 08-10 measured a PAID FREEZE PLATEAU: a robot that freezes at the start height banks ~+120/ep in lower episodes (narrow-kernel income front-loaded in hold+early ramp, PLUS a finish-bonus gate bug: the legacy ref>=target arrival gate is always-open for negative targets, paying the 8mm arrival kernel to a robot still at the TOP), while every imperfect attempt scores below freezing - trying-badly < not-trying < trying-well (+760). Fix, both cfg-gated default-off legacy-exact (smoke: default reward stream md5-identical; freeze return +120 -> -16 with flags on, hold window still paid, arrived-at-target still earns factor 1.0): reward.rise_finish_gate_signed=1 (sign-aware arrival gate) + reward.rise_income_prog_gate=1 (kernel+finish income x fraction-of-target-covered once the ramp departs - same worth-less-by-construction mechanism as walk_kernel_prog_gate). ONE package vs cw-uni-mix0-r1 (same joyjit_dr05 warm start, walk=0 mix, DR0.5, 18M). If-true: rise/lower success fracs climb off zero (pricing was the blocker; re-read the mix-ladder verdicts). If-false: freeze is unpaid AND still 0-success - pricing refuted for the warm-started line, the init itself stands as root cause (measured height-channel blindness: walk-lineage dA/d(height_ref) at unused-channel noise floor 0.22x proprio avg vs stance champion 4.2x; pair with cw-uni-rfix-fresh1).

**gate**: own-cfg DR0.5 rise AND lower success >=5/6 det each by 18M; hold quiet (height_err_end<=8mm); VIDEO: no leg-through-floor; early call permitted if W&B rise/lower success fracs still flat zero at 6M

**verdict**: FAIL its gate (rise 0/12 posture-strict) but fork-deciding. OBSERVATIONS: lower det 6/6 (h_err med 2.2mm, all pads down; sto 4/6) -- FIRST posture-strict lower on the uni-line, off the walk-champion warm start; rise 0/6 det + 0/6 sto: bridge starts finish at height (h_err 4.9-11.4mm) with legs 2&5 held 27-151mm aloft (duty 0.00-0.13, video-confirmed), crouch/flat starts miss height outright (47-58mm); 0 terms. Training reward -64->+59 quarters. INTERPRETATION: the c69/c71 pricing fix (rise_finish_gate_signed + rise_income_prog_gate) demonstrably killed the paid-freeze plateau -- lower is now solved warm. Rise is blocked by a SECOND, now-named pricing hole: rise income+finish pay body height only; end-posture (all feet loaded) is unpriced in training while the harness gates on it, so unloading 2 legs is free and even avoids k_drag_loaded exposure. Chain: flag-legs-at-finish <- height-only income <- finish not gated on end_posture <- no sim defect. VERDICT: keep fine-tune-grafting (distill/two-policy NOT justified: warm start beat from-scratch, see cw-uni-rfix-fresh1); next arm = posture-gated rise finish (gate finish bonus + late income on all-feet-ground-contact, mirroring harness end_posture_ok). HYPOTHESIS STATUS: partially confirmed -- pricing was THE blocker for lower; rise has one more hole.

