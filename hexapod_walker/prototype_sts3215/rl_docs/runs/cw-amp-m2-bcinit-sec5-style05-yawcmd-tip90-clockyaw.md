# cw-amp-m2-bcinit-sec5-style05-yawcmd-tip90-clockyaw

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-22T23:20:30+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05-yawcmd-tip90

**wandb_id**: vz07tkta

**hypothesis**: Plain English: dose twin of tip50-clockyaw -- with the gait clock finally ticking during commanded turns, does maximum turn exposure (90% of episodes) speed up or strengthen turn-in-place learning? Same frozen-clock root cause and fix as the tip50-clockyaw arm (goal.walk_phase_run_on_yaw=1, landed+tested this cycle, default-off bit-exact); byte-identical to tip90 otherwise, so tip90 (parked, tip err 0.2982/0.2999 == |wz_ref|) is the matched control. The original tip50/tip90 pair showed ZERO dose-response because the skill was unlearnable with a frozen clock at any exposure; if the clock was the binding constraint, dose-response should now APPEAR (this arm learns turn-in-place faster/stronger than tip50-clockyaw) -- that pattern would confirm the clock diagnosis twice over. Prediction-if-true: eval_yaw (with the clock key in cfg-set) tip errs drop well below 0.30 with command-signed wz both directions, at least matching tip50-clockyaw. Prediction-if-false: both clockyaw arms still park -- clock refuted as binding constraint, BC-turn-clone next. Translation erosion tolerated on this 0.9-frac arm (stage read, same as tip90).

**gate**: Discovery (2M, DR-0). PASS = eval_yaw manual on the pod with the run cfg INCLUDING goal.walk_phase_run_on_yaw=1 (--speed 0.08 --wz-max 0.3): tip-left AND tip-right err <= 0.20, achieved wz sign matching wz_ref both directions, zero falls (translation erosion tolerated at 0.9 frac; report non-tip-episode gait from the DR-0 panel per-episode -- medians will be ~90% tip-contaminated, see tip50-r2 verdict gotcha). PARTIAL-step = stepping-in-place on tip video with |wz| < 0.1. FAIL-park = tip errs ~0.30 with running clock. Joint dose read with -tip50-clockyaw: clock binding => dose-response appears; both park => clock refuted, BC-turn-clone next.

**verdict**: FAIL-park, clock refuted as the binding constraint. Manual eval_yaw (own cfg incl. goal.walk_phase_run_on_yaw=1, --speed 0.08 --wz-max 0.3): tip-left err 0.2538, tip-right err 0.3315 (turn med 0.2712), hold |wz| 0.0025, 0 falls -- essentially unchanged from the pre-clock-fix parent tip90 (0.2982/0.2999) and from the frac=0.5 twin tip50-clockyaw (0.2898/0.3256). Achieved wz on tip ticks is only ~0.03-0.05 rad/s regardless of commanded sign (fwd-hold left-drift residue, not real rotation) -- the video (walk_det_*_sheet.png) shows legs cycling with high per-tick slip (52-58/m on the 5/6 contaminated tip episodes) but no net turn. DR-0 gate: gait_valid 6/6, non-tip translation episodes (ep4) clean (slip 2.75/m, prog fine), no sacrificed legs -- translation fully preserved, only rotation stayed dead. Root-cause chain closed this cycle: the phase-clock fix (goal.walk_phase_run_on_yaw) was necessary-but-not-sufficient -- with the clock now ticking during pure-turn segments, the policy CAN generate a real gait cycle throughout a tip episode, but the BC-anchored substrate has ZERO turning prior (bc_init_gait.py never drove the teacher's own omega channel), so RL still has to invent rotation from nothing and doesn't at this budget/pricing. Per the pre-registered joint read (both clockyaw arms still park): CLOCK REFUTED, next lever = BC-turn-clone. Built + trained this cycle: rl_move/sim/bc_init_gait.py --drive-omega (bank rl_move/tests/test_bc_init_gait_omega.py 5/5, snapshot below) drives TripodGait's native omega channel during BC data collection; the resulting raw clone (zero RL) already scores eval_yaw tip err 0.096/0.108 -- see cw-amp-m2-turnclone-yawcmd-tip50/-tip90 (PASS/PARTIAL on the same reward stack, launched+read this cycle).

