# cw-walk-joyhead90-r1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T20:50:50+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_joyjit_dr05_c1.zip

**wandb_id**: 8mwr1jrq

**hardware_ready**: False

**hypothesis**: Ruling-7 multi-seed panel for the widest-envelope driving candidate: joyhead90-r1 (c62 PASS, +-90deg abrupt flips at DR0.5, joystick gate 0 falls) is a promotion-relevant driving rung and must be seed-robust before it anchors further composes. One variable off the joyhead90-r1 RECIPE: --seed 0 -> 1 (identical config+parent joyjit-dr05-c1). Prediction-if-true: seed 1 reproduces the pass (joystick gate +-90 zero in-envelope falls, own-cfg DR0.5 gv 12/12, prog med >=0.80) - envelope widening is recipe-robust, not a seed fluke. Prediction-if-false: seed 1 falls on lateral flips or craters progress - the c62 pass was seed luck and the envelope rung needs a heading curriculum. Strongest alternative: passes the letter with visibly worse lateral slip - report the band, panel still counts.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 --heading-max-deg 90 own cfg - ZERO in-envelope falls, left/right dist >=0.15m; own-cfg DR0.5 harness 6+6: gv 12/12, 0 term, prog_ratio med >=0.80; frames watched det

**verdict**: PASS -- seed-1 twin reproduces the joyhead90-r1 pass: envelope widening to +-90deg abrupt flips is recipe-robust, not a seed fluke. JOYSTICK GATE @DR0.2 --heading-max-deg 90: 0 in-envelope falls, left 0.218m/right 0.254m (gate >=0.15), flip-stress trk_err 0.028-0.031 (matches seed0 0.033-0.037). Own-cfg DR0.5 harness det+sto 6/6 @15s: gv 12/12, 0 term, prog med 0.86/0.90 (gate >=0.80) -- matches seed0 (0.87/0.90) almost exactly. Frames det: level six-leg cycling through flips, no flag leg. Ruling-7 seed panel for the widest driving envelope now 2/2. NOTE: trained on default checkpoint name (no --out-name) -- pulled/renamed/md5-verified manually (COMMANDS.md gotcha). Not hardware-ready (paddle slip root).

