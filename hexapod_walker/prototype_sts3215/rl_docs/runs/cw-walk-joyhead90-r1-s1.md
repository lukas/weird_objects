# cw-walk-joyhead90-r1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-09T20:48:45+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_joyjit_dr05_c1.zip

**hypothesis**: Ruling-7 multi-seed panel for the widest-envelope driving candidate: joyhead90-r1 (c62 PASS, +-90deg abrupt flips at DR0.5, joystick gate 0 falls) is a promotion-relevant driving rung and must be seed-robust before it anchors further composes. One variable off the joyhead90-r1 RECIPE: --seed 0 -> 1 (identical config+parent joyjit-dr05-c1). Prediction-if-true: seed 1 reproduces the pass (joystick gate +-90 zero in-envelope falls, own-cfg DR0.5 gv 12/12, prog med >=0.80) - envelope widening is recipe-robust, not a seed fluke. Prediction-if-false: seed 1 falls on lateral flips or craters progress - the c62 pass was seed luck and the envelope rung needs a heading curriculum. Strongest alternative: passes the letter with visibly worse lateral slip - report the band, panel still counts.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 --heading-max-deg 90 own cfg - ZERO in-envelope falls, left/right dist >=0.15m; own-cfg DR0.5 harness 6+6: gv 12/12, 0 term, prog_ratio med >=0.80; frames watched det

**refused_reason**: hexapod-mjx-train-11 code marker d285706807cd1b1a447c3cdbc6f7381a3c690b75 != local HEAD 6bfe059536e29612dfdbb4cebe19498f06dda700. Sync first: snapshot.sh --sync hexapod-mjx-train-11 (and snapshot/commit before that if the tree is dirty).

