# cw-amp-m2-turnclone-yawcmd-tip90

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-23T00:01:09+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05-yawcmd-tip90-clockyaw

**wandb_id**: 8ajjysh1

**hypothesis**: Plain English: the max-exposure (0.9 frac) dose twin of cw-amp-m2-turnclone-yawcmd-tip50 -- with a BC init that was actually TAUGHT to turn (TripodGait's native omega channel driven for the first time, see tip50 sibling's hypothesis for the full root-cause chain and the raw-clone eval_yaw baseline: tip errs 0.096/0.108, near-passing before any RL), does maximum turn-in-place exposure help or hurt vs the 0.5 dose? Single lever vs tip90-clockyaw: --init-from swapped from the yaw-blind yawcmd checkpoint to the turn-clone; everything else (full yaw pricing stack, fresh disc, 2M, frac=0.9) byte-identical. Joint read with the tip50 sibling decides the fork: both clear/beat the raw-clone baseline = turning is real and dose helps or is neutral; tip90 much worse than tip50 = high exposure erodes translation faster than the skill consolidates on this substrate (matches the pre-clone lineage's own dose-neutral finding); both regress to the ~0.28-0.33 park level = REWARD-SIDE lever next regardless of init.

**gate**: Discovery continuation (2M, DR-0). Manual eval_yaw on the run's own pod with the run's own cfg (goal.walk_phase_run_on_yaw=1 included, --speed 0.08 --wz-max 0.3): PASS = tip-left AND tip-right err <=0.20 with achieved wz sign matching wz_ref, zero falls. PARTIAL = improvement over the 0.25-0.33 park fingerprint without fully clearing the gate. FAIL-washout = regresses back to ~0.28-0.33 despite the strong raw-clone start.

**verdict**: PASS on this run's own pre-registered bar, matching its tip50-turnclone sibling. Manual eval_yaw: tip-left err 0.1325, tip-right err 0.1736 (turn med 0.1365), hold |wz| 0.0208, 0 falls. Both directions clear the run's own <=0.20 bar with correct sign, vs the matched-control tip90-clockyaw's 0.25/0.33 park. DR-0 gate (0.9 frac, mostly tip episodes): the 1 non-tip translation episode stayed clean (gait_valid true, slip 2.46/m, progress_ratio 0.625, no sacrificed legs) -- translation survives even at max turn-exposure dose. Joint read with tip50-turnclone (err 0.16/0.13): the two doses are statistically a WASH (tip90 slightly better on left, slightly worse on right) -- turn-in-place exposure dose (0.5 vs 0.9) does not clearly help or hurt once the init actually knows how to turn, matching the pre-clone lineage's own dose-neutral finding on tip50/90 (which never got the chance to show it since both parked identically). Same root cause / single-lever isolation as tip50-turnclone: only --init-from differs from the FAILed tip90-clockyaw control.

