# cw-stand-holdload1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-11T19:59:43+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-crouchrise3

**wandb_id**: oym8yv6a

**hardware_ready**: False

**hypothesis**: MECHANISM TEST (2M, warm from cw-stand-holdbc1-hard1 via inherited init, config = crouchrise3 exactly + ONE new axis): the crouchrise trio park is REWARD-SIDE, not anchor-side. All three doses (0.60, 0.60+mix, 0.45) converged on two legs hovering 1-19mm — under foot_down_mm so the clearance-priced hold_still_gate pays them as down, under flag_leg_mm so no no-flag form fires, while the eval duty clause (touch >0.5N) reads 0.01-0.04. New reward.hold_feet_load=1.0 prices hold/track income on MEASURED touch force: per-foot product of clip(N/1.0,0,1) floored at 0.5, so the two-leg hover earns 0.25 of hold income (FEET-LOAD bank, test_task_semantics.py: hover reproduced at 4-13mm/duty<0.2, pre-fix stack pays it 0.85+ parity, gated stack prices it to scraps, quiet stand untaxed). If the park was profitable only because the gate was blind, 0.45 crouch dose should now keep crouch rises AND the six-foot hold. If it STILL parks at a real 4x income loss, the cheat is not reward-side -- strong evidence for the state-aligned-BC-anchor-bleed mechanism (crouchrise2 suspect) and that spec-first anchor lever proceeds with priority.

**gate**: PASS if det crouch-start rise valid >= 3/4 with zero tilt falls AND hold det+sto valid_plant >= 10/12 AND det-hold per-foot contact duty >= 0.8 on ALL SIX feet AND det flat/bridge rise not worse than hard1 AND det lower success >= 3/6 with <= 1 fall AND no flag-leg/tripod/hover cheat on video. PASS -> replaces hard1 as the stance deploy candidate (ship WITH its goal-ramp profile). FAIL with the same legs-1+4 park (now duty-priced at 0.25 income) -> reward-side hypothesis REFUTED by direct measurement; the anchor-bleed mechanism is the sole remaining suspect and the state-aligned BC anchor spec (crouchrise3 escalation) becomes the only open lever. FAIL any other clause -> triage normally; hold_feet_load itself is validated by the FEET-LOAD bank either way.

**verdict**: Reward-side hypothesis REFUTED (pre-registered branch): reward.hold_feet_load (priced hold income on measured foot touch-force) does NOT stop the legs-1+4 hover-park cheat — det-hold duty_cycle is 0.03/0.04 on legs 1+4 across all 6 det episodes, identical fingerprint to crouchrise1/2/3, despite the new term costing that pattern 0.25x income per the FEET-LOAD bank. valid_plant still reads True (blind to mid-episode duty, known gap). Crouch rise stays clean (det 6/6 incl. crouch 4/4, zero falls). det lower regressed to 2/6 (dangling leg-2, no falls, matches crouchrise3 pattern). Anchor-bleed (state/height-aligned BC anchor) is now the SOLE remaining suspect per direct measurement, not inference; hard1 stays deployed.

