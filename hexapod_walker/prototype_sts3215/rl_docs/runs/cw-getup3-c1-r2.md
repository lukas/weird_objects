# cw-getup3-c1-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-12T12:02:11+00:00

**pod**: hexapod-mjx-train-2

**steps**: 10000000

**parent**: cw-getup3

**wandb_id**: 1pe2a1zd

**hypothesis**: Give the first WORKING getup recipe the steps it was still climbing at the buzzer. cw-getup3 (BC-anchored, warm-started unified recover-stand-walk) ended its 2M discovery budget mid-climb: stand quality env/getup_S 0.09->0.17 monotone, returns positive and rising, height factor 0.31->0.62. Continue from its checkpoint with the identical recipe (zero variables changed; the obs-pad flag is dropped because the parent is already 72-obs) and a 10M budget. Prediction-if-true: S keeps climbing toward 0.3+, reward_getup_hold becomes a visible income line, and video shows sustained supported stands from floor-adjacent starts. Prediction-if-false: S plateaus below 0.2 — the BC anchor can drag the policy near the stand but the S-gated income cannot close the last gap; next lever is stand-income depth (sibling cw-getup4 tests exactly that in parallel). Strongest alternative: the S rise entrenches as a partial tripod pose the fades under-price — visible as feet_loaded stuck ~2.4 while f_height keeps climbing.

**gate**: PASS if env/getup_S ends >0.30 and non-declining, mean reward_getup_hold >0.05, and video shows at least one sustained (>3 s) supported stand from a floor-adjacent start with no flag-leg/stilt exploit; FAIL if S plateaus <0.2 by 6M or the video shows static collapse or an exploit dominating.

