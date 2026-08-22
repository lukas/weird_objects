# cw-amp-m2-bcinit-sec5-style05-speedrange-fastphase-nostyle

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-22T22:27:55+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05-speedrange

**wandb_id**: hfvmy28b

**hypothesis**: Plain English: paired sibling to -fastphase, testing WHICH mechanism caps achieved speed at ~0.08-0.14 m/s despite a 0.05-0.25 m/s command range. Same single lever (goal.walk_phase_hz 1.333333 -> 2.0) but ALSO zeroes the AMP style weight (amp-task-weight 0.5->1.0, amp-style-weight 0.5->0.0) from the same speedrange checkpoint -- removes any pull from the discriminator toward the teacher_v2 library's ORIGINAL fixed-cadence clips while keeping every other lever (task reward, obs, faster clock) identical to the -fastphase sibling. Prediction-if-true (style is the cap): this arm's speed_mean spread widens past 0.08-0.14 while the style-keeping -fastphase sibling does not -- the discriminator was vetoing faster-than-demonstrated stepping. Prediction-if-false (style is not the cap): both arms behave the same (either both widen or both stay pinned) -- the cadence cap lives elsewhere (phase_hz itself, or actor/PPO capacity at this budget), not in AMP style pricing. Strongest alternative: removing style also destabilizes gait quality (paddle-creep/drag returns) even if speed widens -- a real task/style tradeoff, not a free fix.

**gate**: Discovery continuation (2M, DR-0, judged jointly with the -fastphase sibling per the hypothesis above). INFORMATIVE-PASS = gait_valid stays >=5/6 det+sto, no new sacrificed legs/terminations, AND speed_mean spread widens past the parent's ~0.08-0.14 m/s band. FAIL-collapse = terminations/sacrificed legs/statue, OR gait quality regresses to drag/paddle-creep even if speed nominally rises (video-overrides-scalar). NO-CHANGE = matches -fastphase's own reading (both widen or both stay pinned) -- decides whether AMP style is the cap or not.

**verdict**: Removing the AMP style reward entirely does NOT free the robot to track speed commands - the discriminator was never the speed cap. This twin ran the same faster clock (walk_phase_hz 2.0) with amp-task-weight=1.0/style-weight=0.0: gait stayed healthy (DR-0 gate gait_valid 6/6 det + 6/6 sto, zero terminations, clean six-leg cycling and real translation on the det strip; slip med 3.86 det, in family with the styled twin's 4.12) yet realized speed_mean spans 0.074-0.118 det / 0.090-0.113 sto against 0.053-0.171 commanded - statistically indistinguishable from the styled sibling and no wider than the speedrange parent (0.084-0.136). Joint pre-registered read = NO-CHANGE branch: neither the style channel (anchored to teacher_v2's original cadence) nor a uniformly faster clock is the cap. Reward rose monotonically 55->241 while vel_err stayed large at high commands - per the 08-21 ruling that is a mechanism/alignment defect, not undertraining: the phase clock advances at a FIXED rate regardless of commanded speed, so cadence cannot follow the command by construction. Next: implement speed-coupled phase clock (cfg-gated, default OFF, semantics-bank test first), continue from the speedrange checkpoint with coupling ON.

