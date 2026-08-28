# cw-standwalk-unified1-joyfix-hdg-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-28T16:06:15+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-standwalk-unified1-mix-long-s0

**wandb_id**: lve8e2nw

**hypothesis**: Plain English: the unified walking policy barely responds to sideways joystick commands because training never draws them — the recipe pins every command heading to straight-ahead (walk_heading_max_rad=0.0), so lateral commands only ever appear via the square/sweep schedule rotations and diagonals never (08-28 audit: crab response ~7% of forward, frozen-state action deltas for diagonals 3x smaller than cardinals, plumbing verified healthy). This canary opens the command-heading draw to the full circle on the otherwise identical 60s mixed-session recipe, warm from long-s0's own 16M PASS checkpoint. Predict-if-true: at 2M the lateral (left/right) net-displacement speed ratio in the local response-matrix probe at least doubles from 0.033 while forward stays >=0.35 (parent 0.475) and session terminations do not explode. Predict-if-false: lateral stays ~0 or forward/session health collapses (would say lateral needs a staged heading-set curriculum, not full-circle at once). Strongest alternative: heading exposure alone is insufficient because the velocity-blind mode-2 obs can't support closed-loop lateral tracking (the velobs3/bundle arms decide this).

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. CANARY (mechanism health only, no behavior maturity claim): at 2M — training reward not collapsing (no sustained fall vs the long-s0 continuation trend at matched steps), DR-0 det walk gait_valid stays >=5/6 with no new sacrificed leg, session eval terminations not exploding (>2x parent's 0/90 baseline band = any >6/90 fails), AND probe_cmd_sensitivity response matrix on the 2M ckpt: max(left,right) speed_ratio >=0.07 (2x parent 0.033) with fwd >=0.35. PASS -> 2-seed acquisition; FAIL with healthy training -> staged heading-set curriculum arm instead.

