# cw-stand-footz1-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-12T09:07:03+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-stand-margin1

**wandb_id**: yn076fe6

**hypothesis**: Make the robot stop holding its stand with one foot secretly hovering in the air, by teaching the imitation loss to see foot HEIGHT instead of only joint angles. Today's audit proved the six-run parked-foot habit was never taught and never defied — it is simply INVISIBLE to the joint-space anchor (the parked leg's supervision loss, 0.0032, matches the clean parent's 0.0031: a mm-scale hover is fractions of a degree across 3 of 18 dims) and every per-foot price finds parking as the escape valve (an airborne foot pays nothing). One variable vs holdbc1-hard1: train.bc_anchor_foot_z 0 -> 1.0 (new FK foot-height anchor term, default-off bit-exact, bank-pinned to charge a 10mm hover >=50x its joint-MSE cost). (Name note: -r1 because the exp/cw-stand-footz1 tag was consumed by this cycle's pre-launch snapshot; no run ever trained under the base name.) Prediction-if-true: det hold keeps ALL SIX feet at duty >=0.5 (the fingerprint every prior arm failed) with rise/lower retention clean — a deployable stance candidate and the first supervision signal that can even see the exploit. Prediction-if-false: park persists with train/bc_anchor_footz_loss LOW (the policy tracks commanded heights but breaks contact through body pose, not leg lift — term supervises the wrong frame) or footz_loss HIGH and undescending (PPO outmuscles it — coefficient question, one dose retry then stop). Strongest alternative: the park moves to a NEW evasion (body tilt unloading a foot at correct commanded height), which the foot-load duty gate still catches.

**gate**: PASS if det hold: every foot duty >=0.5 in all 6 episodes AND hold det+sto valid_plant 12/12 AND det rise >=5/6 valid_plant with no new falls AND lower det >= matched-parent probe (4/6 baseline this seed) with NO outrigger/flag-leg on video AND drag_m/roll_tail quoted vs parent (no worsening beyond noise). FAIL if any foot parks (duty <0.5 det hold) or retention regresses; record train/bc_anchor_footz_loss trajectory either way.

