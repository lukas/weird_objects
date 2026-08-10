# cw-walk-posetrack-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T00:59:38+00:00

**pod**: hexapod-mjx-train-0

**steps**: 15000000

**parent**: cw-walk-posetrack

**wandb_id**: xbsqmrfu

**hardware_ready**: False

**hypothesis**: Continuation of cw-walk-posetrack (FAIL-letter, 10M steps): hold nailed (0.84deg) but lean/track missed the 1.5deg gate at 2.2-3.6deg with 0 terminations and no leg-through-floor -- looked like undertraining on a genuinely new skill, not a hard ceiling (same mix already skews 0.4/0.4 lean/track vs 0.2 hold). ONE variable: warm-start from posetrack's own checkpoint (not the walk champion) and add 15M more steps, same mix/config otherwise. If-true: track_err_mean_deg drops to <=1.5 on most det/sto episodes, 0 term, video shows stable planted stance while tracking. If-false (still >1.5deg after 25M total): budget isn't the lever -- lean/track needs either a denser tracking reward or a curriculum (start near-level, widen), not more steps.

**gate**: own-cfg det+sto 6/6 each mode (hold/lean/track): track_err_mean_deg<=1.5 AND lean_err_mean_deg<=1.5 on majority of episodes, 0 term, height_err<=10mm, VIDEO: no leg-through-floor, no falls; frames watched det

**verdict**: FAIL (if-false effectively confirmed: budget was not the lever). +15M steps warm-started from posetrack itself moved hold and lean but NOT track. hold: nailed, 12/12 episodes <=1.5deg (med 0.83-0.92deg), 0 term -- unchanged strength. lean: improved from the parents FAIL band but still short -- only 6/12 episodes (det+sto) meet <=1.5deg (det median 1.31deg, sto median 2.18deg), not the majority the gate needs. track: barely moved, 1/12 episodes <=1.5deg (det median 1.9-2.0deg, sto median ~2.0deg) -- essentially the same failing band as the 10M-step parent. height_err_end_mm stayed tight (2-10.8mm, one episode 0.8mm over the 10mm cap, trivial). 0 terminations throughout; det frames on both hold and track show a stable planted six-leg stance, body leaning/tilting on command with no leg sweeping through the floor -- the pathology is pure tracking-accuracy shortfall, not instability. Conclusion: more steps helped lean but did nothing for track -- confirms the pre-registered alternative that lean/track need a denser tracking reward or curriculum, not a bigger step budget. No further steps-only continuation of this exact recipe.

