# cw-uni-rfix-warm1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T13:51:24+00:00

**pod**: hexapod-mjx-train-0

**steps**: 18000000

**parent**: cw-uni-mix0-r1

**wandb_id**: ln94x5rr

**hypothesis**: Reward-pricing arm of the c69/c71 uni-line DIG-IN (the rise/lower income audit pre-registered in blend1-r2's if-false). Root-cause audit 08-10 measured a PAID FREEZE PLATEAU: a robot that freezes at the start height banks ~+120/ep in lower episodes (narrow-kernel income front-loaded in hold+early ramp, PLUS a finish-bonus gate bug: the legacy ref>=target arrival gate is always-open for negative targets, paying the 8mm arrival kernel to a robot still at the TOP), while every imperfect attempt scores below freezing - trying-badly < not-trying < trying-well (+760). Fix, both cfg-gated default-off legacy-exact (smoke: default reward stream md5-identical; freeze return +120 -> -16 with flags on, hold window still paid, arrived-at-target still earns factor 1.0): reward.rise_finish_gate_signed=1 (sign-aware arrival gate) + reward.rise_income_prog_gate=1 (kernel+finish income x fraction-of-target-covered once the ramp departs - same worth-less-by-construction mechanism as walk_kernel_prog_gate). ONE package vs cw-uni-mix0-r1 (same joyjit_dr05 warm start, walk=0 mix, DR0.5, 18M). If-true: rise/lower success fracs climb off zero (pricing was the blocker; re-read the mix-ladder verdicts). If-false: freeze is unpaid AND still 0-success - pricing refuted for the warm-started line, the init itself stands as root cause (measured height-channel blindness: walk-lineage dA/d(height_ref) at unused-channel noise floor 0.22x proprio avg vs stance champion 4.2x; pair with cw-uni-rfix-fresh1).

**gate**: own-cfg DR0.5 rise AND lower success >=5/6 det each by 18M; hold quiet (height_err_end<=8mm); VIDEO: no leg-through-floor; early call permitted if W&B rise/lower success fracs still flat zero at 6M

