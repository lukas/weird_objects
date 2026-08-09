# cw-walk-strafe

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T12:43:39+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: cw-walk-anchorgate

**wandb_id**: vb3hg2bu

**hardware_ready**: no

**hypothesis**: OPERATOR WISHLIST 5: omnidirectional walking. Heading commands widened to +-90deg (full lateral strafing) with resampling; the champion only ever saw +-45deg. If-true: lateral tracking with gait_valid (omnidirectional base for the driving demo); if-false: sideways gait degenerates (hexapod kinematics may prefer crab-diagonal over pure strafe).

**gate**: DR0 det+sto 6/6 incl pure-lateral commands: gait_valid, zero terminations, lateral tracking err <= 2x forward err

**verdict**: PASS (exploratory, qualified). ±90° lateral commands transport for real: prog_ratio 0.81-1.21 in all 12 eps, gv 12/12, 0 term, lateral vel_err 0.031-0.037 vs fwd ~0.028 (within the ≤2x gate). Frames: six feet cycling, level body, no flag legs — but it strafes by paddling at slip/m ~2.0 (2x fwd champion). Lateral omni base for the driving demo exists; rear hemisphere remains broken (see backforth). NOT hardware-ready (slip).

