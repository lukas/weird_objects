# cw-getup3-c2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-12T03:01:23+00:00

**pod**: hexapod-mjx-train-2

**steps**: 10000000

**parent**: cw-getup3

**wandb_id**: tbi5ukgc

**hypothesis**: Give the first WORKING getup recipe the steps it was still climbing at the buzzer (cw-getup3-c1's own goal, relaunched clean after a launch-config bug ate the first attempt with 0 training steps). cw-getup3 (BC-anchored, warm-started unified recover-stand-walk) ended its 2M discovery budget mid-climb: stand quality env/getup_S 0.09->0.17 monotone, returns positive and rising, height factor 0.31->0.62. Continue from its checkpoint with the IDENTICAL recipe (zero variables changed, and this time obs-pad-transplant correctly OFF since this continuation has no obs widening) and a 10M budget. Prediction-if-true: S keeps climbing toward 0.3+, reward_getup_hold becomes a visible income line, and video shows sustained supported stands from floor-adjacent starts. Prediction-if-false: S plateaus below 0.2 -- the BC anchor can drag the policy near the stand but the S-gated income cannot close the last gap; next lever is stand-income depth. Strongest alternative: the S rise entrenches as a partial tripod pose that fades under-price -- visible as feet_loaded stuck ~2.4 while f_height keeps climbing.

**gate**: PASS if env/getup_S ends >0.30 and non-declining, mean reward_getup_hold >0.05, and video shows at least one sustained (>3 s) supported stand from a floor-adjacent start with no flag-leg/stilt exploit; FAIL if S plateaus <0.2 by 6M or the video shows static collapse or an exploit dominating.

