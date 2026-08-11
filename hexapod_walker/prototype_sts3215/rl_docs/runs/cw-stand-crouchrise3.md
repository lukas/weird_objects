# cw-stand-crouchrise3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-11T18:35:47+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-crouchrise2

**wandb_id**: jjha6hzd

**hardware_ready**: no

**hypothesis**: DISCOVERY (2M, warm from cw-stand-holdbc1-hard1 via inherited init, NOT from crouchrise2 per its verdict): crouch-start DOSE response. The hard1 stack at the legacy 25 percent crouch share never learns crouch rises (0/8 RSI-off); crouchrise1/2 at 60 percent both learn them (crouch 5/5 and 4/4) and BOTH flip det-hold into the identical legs-1+4 park (duty 0.03/0.03) with hold pricing already maxed (hold_still_gate=1.0, hold_flag_fade=1.0) - so goal-mix skew is refuted as the cause (crouchrise2) and the remaining config suspect is the start-mix dose itself. crouchrise3 = ONE axis vs crouchrise2: crouch share 0.60 -> 0.45 (rise_flat_frac 0.10->0.15, rise_partial_frac 0.30->0.40), everything else identical. Tests whether a mid dose keeps the crouch-rise fix without pushing det-hold off the six-foot optimum.

**gate**: PASS if det crouch-start rise valid >= 3/4 with zero tilt falls AND hold retention at hard1 level: hold det+sto valid_plant >= 10/12 AND det-hold per-foot contact duty >= 0.8 on ALL SIX feet (explicit) AND det flat/bridge rise not worse than hard1 AND det lower success >= 3/6 with <= 1 fall (the hard1 matched-control band; crouchrise2 regressed lower to 0/6 so it now gates explicitly) AND no flag-leg/tripod cheat on video. FAIL if crouch <= 2/4 or any hold/duty/lower clause regresses. PASS -> replaces hard1 as the stance deploy candidate; the robot export MUST ship WITH its goal-ramp profile. FAIL at 0.45 with the same legs-1+4 park -> the dose axis is dead; escalate to the rise-teaching lever (state-aligned BC anchor / seeding) pending the risectl1 fork verdict.

**verdict**: FAIL per the pre-registered 0.45-dose branch — the crouch-start DOSE axis is DEAD. Crouch rise is retained clean (det crouch 4/4 valid_plant + flat 1/1 + bridge 1/1, zero falls; sto rise 5/6) but det-hold parks the IDENTICAL two legs as crouchrise1 (0.07/0.01) and crouchrise2 (0.03/0.03): feet 1+4 contact duty 0.04/0.01 vs hard1's all-six 0.90-0.99, with valid_plant reading True (end_clear 1-2mm masks the hover; the explicit duty clause catches it). Hold det+sto valid_plant 9/12 (<10/12 gate; sto misses all >2A current soft-flags, Imax 2.52-2.63) and det lower 2/6 (<hard1's 3/6 band, zero falls, same 73-99mm dangling-foot incentive the lower bank xfail documents). Three doses (0.60, 0.60+mix-restore, 0.45) now reproduce the same flag-leg hold; the cheat follows ANY elevated crouch-start share, not its size, mix, or step count. hard1 STAYS the stance deploy candidate; do not deploy or warm from crouchrise3. Next lever is CODE with SPECIFICATION first: state/height-aligned BC anchor for non-flat rise starts (the clock-indexed anchor teaches lifted-leg reference poses in plant-adjacent states on crouch starts and it bleeds into hold), per the crouchrise2 mechanism suspect.

