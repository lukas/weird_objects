# cw-walk-wander60-dr05-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-09T21:36:36+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_wander_dr05.zip

**hardware_ready**: False

**hypothesis**: Ruling-7 seed twin of cw-walk-wander60-dr05 (c62 PASS: minute-long driving at DR0.5, gv 12/12, prog med 0.94/0.93, no late-episode degradation). The 60s x DR0.5 convergence rung is the closest sim rung to real operator joystick sessions and should be recipe-robust before anchoring anything. One variable off the wander60-dr05 RECIPE: --seed 0 -> 1 (identical config+parent wander-dr05). Prediction-if-true: seed 1 reproduces the pass (own-cfg DR0.5 60s gv 12/12, 0 term, prog med >=0.85, slip/m med <=2.4) - minute-long robust driving is seed-robust like the wander-dr05 3-seed panel. Prediction-if-false: seed 1 shows late-episode stalls/slip growth - the c62 pass was seed luck and long-horizon robustness needs recovery-focused training. Strongest alternative: letter pass with more low-prog sto eps - report the tail, panel still counts.

**gate**: own-cfg DR0.5 60s 6+6 resampled cmds: gv 12/12, 0 term, prog_ratio med >=0.85, no ep prog <0.5, slip/m med <=2.4, worst-ep <=2.6; DR0 det retention 6 eps gv 6/6; frames watched det full horizon

**verdict**: FAIL on the pre-registered gv-12/12 gate: seed-1 twin of the wander60-dr05 60s/DR0.5 driving PASS (c62) reproduces cleanly in det mode (6/6 gv both DR0.5-own-cfg and DR0-retention, prog med 0.95-0.98, slip/m med 1.16-1.35) but sto/0 has a genuine sacrificed leg (leg 4 duty 0.08 vs siblings 0.37-0.89, slip/m 2.11, video-checked, not a march-in-place false-positive) -- 11/12 not 12/12. Informative negative: refutes strict seed-robustness for THIS long-horizon (60s) DR0.5 rung under stochastic sampling; det/hardware-relevant mode is unaffected. Ledger note: checkpoint was saved on-pod under the trainer default name (ppo_mjx_joint_walk_cw-walk-wander60-dr05-s1.zip, no --out-name set at launch) not the ppo_goal_ convention -- pulled+renamed manually (md5 52315ebe), no harness eval had run before this cycle.

