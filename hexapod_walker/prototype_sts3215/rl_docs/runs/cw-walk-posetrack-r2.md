# cw-walk-posetrack-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T00:59:38+00:00

**pod**: hexapod-mjx-train-0

**steps**: 15000000

**parent**: cw-walk-posetrack

**wandb_id**: xbsqmrfu

**hypothesis**: Continuation of cw-walk-posetrack (FAIL-letter, 10M steps): hold nailed (0.84deg) but lean/track missed the 1.5deg gate at 2.2-3.6deg with 0 terminations and no leg-through-floor -- looked like undertraining on a genuinely new skill, not a hard ceiling (same mix already skews 0.4/0.4 lean/track vs 0.2 hold). ONE variable: warm-start from posetrack's own checkpoint (not the walk champion) and add 15M more steps, same mix/config otherwise. If-true: track_err_mean_deg drops to <=1.5 on most det/sto episodes, 0 term, video shows stable planted stance while tracking. If-false (still >1.5deg after 25M total): budget isn't the lever -- lean/track needs either a denser tracking reward or a curriculum (start near-level, widen), not more steps.

**gate**: own-cfg det+sto 6/6 each mode (hold/lean/track): track_err_mean_deg<=1.5 AND lean_err_mean_deg<=1.5 on majority of episodes, 0 term, height_err<=10mm, VIDEO: no leg-through-floor, no falls; frames watched det

