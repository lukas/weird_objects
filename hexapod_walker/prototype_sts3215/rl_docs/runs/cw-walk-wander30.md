# cw-walk-wander30

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T13:50:30+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_wander.zip

**wandb_id**: e4wuad6b

**hardware_ready**: no

**hypothesis**: Driving endurance: cw-walk-wander PASSED start/steer/stop transitions over 15s (~3 command changes). One variable off it: horizon 15->30s (~6 changes/ep). Plain: prove the drive-it-around skill doesn't decay over longer drives. Prediction-if-true: 30s eps hold prog_ratio ~1.0 with gv and no term across all command changes. Prediction-if-false: degradation accumulates late-episode (height sag, slip growth, parked segments after later changes) -> transitions are only locally stable. Strongest alternative: fine at 30s but slip on change-segments stays ~2x straight-line (contact-pricing root, not fixable here).

**gate**: own-cfg DR0 30s 6+6: gv 12/12, 0 term, prog_ratio median 0.85-1.15, no ep prog<0.5; change-segment slip no worse than parent (~2.1 slip/m ep-level); frames watched det+sto

**verdict**: PASS. 30s driving endurance (±45° resample/5s, 15% stops): own-cfg DR0 gv 12/12, 0 term, prog 0.94-1.02 (gate 0.85-1.15, no ep <0.5), worst-ep slip/m 1.93 vs parent change-segment ~2.1 — no late-episode degradation across ~6 command changes/ep. Frames det+sto: level six-leg cycling through heading changes, no flag leg. Notably zero draw-stall eps in this panel (command resampling appears to break stall draws). hardware-ready: no (contact-pricing slip root still open).

