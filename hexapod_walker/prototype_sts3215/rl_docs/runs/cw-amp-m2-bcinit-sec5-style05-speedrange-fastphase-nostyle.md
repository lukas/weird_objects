# cw-amp-m2-bcinit-sec5-style05-speedrange-fastphase-nostyle

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T22:27:55+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05-speedrange

**wandb_id**: hfvmy28b

**hypothesis**: Plain English: paired sibling to -fastphase, testing WHICH mechanism caps achieved speed at ~0.08-0.14 m/s despite a 0.05-0.25 m/s command range. Same single lever (goal.walk_phase_hz 1.333333 -> 2.0) but ALSO zeroes the AMP style weight (amp-task-weight 0.5->1.0, amp-style-weight 0.5->0.0) from the same speedrange checkpoint -- removes any pull from the discriminator toward the teacher_v2 library's ORIGINAL fixed-cadence clips while keeping every other lever (task reward, obs, faster clock) identical to the -fastphase sibling. Prediction-if-true (style is the cap): this arm's speed_mean spread widens past 0.08-0.14 while the style-keeping -fastphase sibling does not -- the discriminator was vetoing faster-than-demonstrated stepping. Prediction-if-false (style is not the cap): both arms behave the same (either both widen or both stay pinned) -- the cadence cap lives elsewhere (phase_hz itself, or actor/PPO capacity at this budget), not in AMP style pricing. Strongest alternative: removing style also destabilizes gait quality (paddle-creep/drag returns) even if speed widens -- a real task/style tradeoff, not a free fix.

**gate**: Discovery continuation (2M, DR-0, judged jointly with the -fastphase sibling per the hypothesis above). INFORMATIVE-PASS = gait_valid stays >=5/6 det+sto, no new sacrificed legs/terminations, AND speed_mean spread widens past the parent's ~0.08-0.14 m/s band. FAIL-collapse = terminations/sacrificed legs/statue, OR gait quality regresses to drag/paddle-creep even if speed nominally rises (video-overrides-scalar). NO-CHANGE = matches -fastphase's own reading (both widen or both stay pinned) -- decides whether AMP style is the cap or not.

