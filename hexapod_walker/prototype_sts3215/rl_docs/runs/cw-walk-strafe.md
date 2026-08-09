# cw-walk-strafe

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T12:43:39+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: cw-walk-anchorgate

**wandb_id**: vb3hg2bu

**hypothesis**: OPERATOR WISHLIST 5: omnidirectional walking. Heading commands widened to +-90deg (full lateral strafing) with resampling; the champion only ever saw +-45deg. If-true: lateral tracking with gait_valid (omnidirectional base for the driving demo); if-false: sideways gait degenerates (hexapod kinematics may prefer crab-diagonal over pure strafe).

**gate**: DR0 det+sto 6/6 incl pure-lateral commands: gait_valid, zero terminations, lateral tracking err <= 2x forward err

