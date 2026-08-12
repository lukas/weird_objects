# cw-stand-riserock3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-12T01:12:43+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-riserock2-r1

**wandb_id**: fnmzg2ve

**hardware_ready**: False

**hypothesis**: Teach the robot to stay level while its body is forced to rock sideways during standing up, at a gentler dose than the last try. cw-stand-riserock2-r1 (same rise-rock DR, dose prob=0.5/deg=6-12) showed ZERO measurable resilience versus a checkpoint that never trained on this at all when both were tested at the exact hardware trip angle (10 deg, guaranteed every episode) — both totally failed. This is the gate's own pre-registered retry: shrink the top of the angle range from 12 to 10 (same probability), so the 10-degree bench threshold sits at the EDGE of what training saw instead of in the middle of an 6-12 range where lower/easier draws diluted the signal. Prediction-if-true: rocked det rise clears the same 10-deg gate this time (>=5/6 valid_plant, zero tilt falls) with hard1/holdbc1 still failing it (matched-parent separation appears). Prediction-if-false: still zero separation -> per the two-miss rule this closes the whole command-bias roll-injection family (rise-rock AND walk-kick both null) and the next lever is contact/pinning modeling, not another dose.

**gate**: PASS if injected eval (eval_checkpoint --cfg-set dr.rise_rock_prob=0.5 --cfg-set dr.rise_rock_deg=6,10 --baseline ppo_goal_cw_stand_holdbc1_hard1.zip, det) rise valid_plant >= 5/6 with ZERO tilt falls AND the baseline under the IDENTICAL injection fails >= 2/6 (matched-parent control) AND nominal det rise/hold matches riserock2-r1's own retention (rise/hold/lower 6/6, no duty regression). PASS -> export candidate, queue a hardening pass before any bench retry. FAIL (still zero separation) -> CLOSE the command-bias roll-injection family (2nd axis, 2nd no-separation dose) per the two-miss rule; next lever is contact/pinning modeling (belly/foot geometry), not another dose.

**verdict**: FAIL -- retention regression pre-empts the rise-rock question, no forensics needed. Own-mix det LOWER collapsed from riserock2-r1's clean 6/6 (worst_clear <=46mm) to 1/6 (worst_clear up to 126mm on 5/6 eps): video shows a NEW three-leg flag-leg/outrigger cheat (legs 1/3/5 duty ~0.9 fully planted, legs 0/2/4 duty 0.05-0.14 stay splayed 10-126mm off the ground, every episode) -- exactly the flag-leg/outrigger class the LOWER MDP_PREFLIGHT bank already names as a known cheat. Known exploit in video = complete verdict per RUN_INTERPRETATION_RULES; this alone breaks the gate's mandatory 'nominal rise/hold/lower retention, no duty regression' clause, so PASS is unreachable regardless of the rise-rock injection result (own-mix det rise 5/6 no falls, roll tail <2deg, looked fine in isolation but moot). Per the two-miss rule (riserock2-r1 zero-separation null + this gentler dose's own retention regression), the command-bias rise-rock DR family is now CLOSED: neither a harder nor a gentler dose bought resilience, and the gentler one broke something else. hard1 stays deployed; do not warm-start/deploy from riserock3. Next lever for the hardware belly-curl rocking gap is contact/pinning modeling (belly/foot contact geometry), not another DR dose.

