# cw-amp-m2-turnclone-yawcmd-tip90

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-23T00:01:09+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05-yawcmd-tip90-clockyaw

**hypothesis**: Plain English: the max-exposure (0.9 frac) dose twin of cw-amp-m2-turnclone-yawcmd-tip50 -- with a BC init that was actually TAUGHT to turn (TripodGait's native omega channel driven for the first time, see tip50 sibling's hypothesis for the full root-cause chain and the raw-clone eval_yaw baseline: tip errs 0.096/0.108, near-passing before any RL), does maximum turn-in-place exposure help or hurt vs the 0.5 dose? Single lever vs tip90-clockyaw: --init-from swapped from the yaw-blind yawcmd checkpoint to the turn-clone; everything else (full yaw pricing stack, fresh disc, 2M, frac=0.9) byte-identical. Joint read with the tip50 sibling decides the fork: both clear/beat the raw-clone baseline = turning is real and dose helps or is neutral; tip90 much worse than tip50 = high exposure erodes translation faster than the skill consolidates on this substrate (matches the pre-clone lineage's own dose-neutral finding); both regress to the ~0.28-0.33 park level = REWARD-SIDE lever next regardless of init.

**gate**: Discovery continuation (2M, DR-0). Manual eval_yaw on the run's own pod with the run's own cfg (goal.walk_phase_run_on_yaw=1 included, --speed 0.08 --wz-max 0.3): PASS = tip-left AND tip-right err <=0.20 with achieved wz sign matching wz_ref, zero falls. PARTIAL = improvement over the 0.25-0.33 park fingerprint without fully clearing the gate. FAIL-washout = regresses back to ~0.28-0.33 despite the strong raw-clone start.

