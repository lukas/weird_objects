# cw-stand-anchorstate1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-11T20:37:48+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-holdload1

**wandb_id**: 4lcfepom

**hypothesis**: ANCHOR-BLEED FIX (2M, warm from cw-stand-holdbc1-hard1 via inherited init; config = holdload1 exactly + ONE new axis, so the load-priced hold gate stays on as defense-in-depth): crouchrise1/2/3 + holdload1 eliminated every reward-side suspect -- holdload1 kept the legs-1+4 hover-park at a measured 4x hold-income loss, so the pose is TAUGHT by the BC anchor, not paid for by the reward. Mechanism (probe-confirmed): non-RSI crouch starts time-align the anchor clock at the BELLY ramp start, so plant-adjacent states are supervised toward early-path lifted-leg poses (first crouch-start target sits 78deg RMS from the robot pose, on the lifted-leg early path) and pi generalizes that obs->action association into hold. train.bc_anchor_state_aligned=1.0 re-indexes the anchor every tick by nearest reference pose to the CURRENT joints + 0.25s pursuit lookahead: crouch/plant-adjacent states can only ever anchor toward the planted tail (probe: target exactly the ref end pose), sagged states get climb-from-where-you-are. Pinned in test_bc_anchor.py (legacy bleed documented, plant-tail fix, forward chain progress). If the anchor bleed was the cheat mechanism, 0.45 crouch dose should now deliver crouch rises AND a six-foot det-hold. If the park STILL appears with the anchor supervising planted poses in plant-adjacent states AND a 4x income penalty on it, the unified stand line has no remaining identified mechanism and falls back to specialist-handoff composition (already deployed for attempt #2).

**gate**: PASS if det crouch-start rise valid >= 3/4 with zero tilt falls AND hold det+sto valid_plant >= 10/12 AND det-hold per-foot contact duty >= 0.8 on ALL SIX feet AND det flat/bridge rise not worse than hard1 AND det lower success >= 3/6 with <= 1 fall AND no flag-leg/tripod/hover cheat on video. PASS -> replaces hard1 as the stance deploy candidate (ship WITH its goal-ramp profile); also close RL_PLAN queue 2a state-alignment as CONFIRMED. FAIL with the same legs-1+4 park -> anchor-bleed REFUTED as the sole mechanism (with reward-side already refuted by holdload1): the unified stand line is out of identified levers -- record it dead, stance stays hard1 + specialist handoff. FAIL any other clause (e.g. rise regression from the state-aligned targets) -> triage normally; the lookahead (bc_anchor_lookahead_s) is the first knob to inspect.

