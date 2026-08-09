# cw-walk-terrain10

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T15:01:49+00:00

**pod**: hexapod-mjx-train-7

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_terrain05.zip

**wandb_id**: 17wn3zcs

**hardware_ready**: no

**hypothesis**: Terrain rung 2: amp 0.5 -> 1.0 (36mm bumps), one variable off cw-walk-terrain05 (PASS at 0.5 with zero flat regression). Plain: find the bump height where the gait needs actual adaptation. If-true: own-cfg amp1.0 det 6/6 gv 0 term with flat retention intact — gait absorbs 2x bumps. If-false: terminations/tilt or flat slip regression past 1.35 — amp 1.0 needs a ramp curriculum or foot-height changes. Strongest alternative: amp 1.0 is still trivially crossable (operator preview: champion paddles across amp 1.0 unimpeded) and the arm teaches nothing new — then the terrain line closes as saturated, not failed.

**gate**: non-promotion exploratory: own-cfg terrain(amp1.0) det 6/6 gv 0 term; flat DR0 det 6/6 gv 0 term slip/m <=1.35; frames watched on terrain

**verdict**: PASS (exploratory gate met) + LINE CLOSED AS SATURATED: own-cfg terrain amp1.0 (36mm bumps) det 6/6 gv, 0 term, prog 1.06, slip/m 0.94; flat DR0 retention det 6/6 gv, 0 term, slip/m 0.95 (gate <=1.35). Terrain and flat metrics are IDENTICAL (prog 1.06 both, slip 0.94/0.95) and match parent terrain05 — amp 1.0 is trivially crossable, exactly the pre-registered strongest-alternative. Frames on terrain: level, six legs cycling, no flag leg. Nothing left to learn from hfield amplitude; harder ground needs the [CODE] obstacle/scene work (wishlist 13d/24). hardware-ready: no.

