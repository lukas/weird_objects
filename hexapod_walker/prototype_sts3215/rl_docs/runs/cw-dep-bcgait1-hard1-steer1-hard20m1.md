# cw-dep-bcgait1-hard1-steer1-hard20m1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-18T16:39:50+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-dep-bcgait1-hard1-steer1c

**wandb_id**: w3fbxfj7

**hardware_ready**: False

**hypothesis**: Teach the tall walker to survive abrupt joystick direction changes for real, not just survive a 2M mechanism check: continue the exact steer1c recipe (120s stress-mix episodes, all six command-schedule families, instant no-blend switches, irregular 2-20s dwells) to ~20M steps with a --best-ckpt retention guard so the run can never publish a checkpoint worse than its own best. Ordered lineage: operator fb_20260818T152717_278879's direction-switch fix, now the pre-registered follow-up named in hw/STATUS.md WAITING-ON. Prediction-if-true: at 20M the checkpoint retains the fixed-command hard1 quality (height >= -20mm, six-leg gait-valid, zero falls, slip <= 1.8/m) AND passes long det+sto direction-switch panels with zero falls/tangles, all legs cycling after every command change, no accumulating yaw-limit saturation, and prompt tracking recovery post-switch. Prediction-if-false: the longer stress-mix exposure either erodes the base fixed-command gait (retention loss) or the tangle/saturation symptoms persist despite the exposure, meaning the fix needs a staged-dwell curriculum instead of the full mix from step 0.

**gate**: HARDENING: --best-ckpt retention guard active throughout. PASS requires ALL: (1) fixed-command panel (dr-scale 0, no switches) matches or beats hard1: height >= -20mm, six-leg gait-valid every episode, zero falls, slip <= 1.8/m; (2) long det+sto direction-switch panels (probe_dirswitch_tangle-style, both DR-0 and own-DR) show zero falls/tangles; (3) all six legs resume cycling within a bounded number of ticks after every command change (no multi-second freeze); (4) no accumulating yaw-limit saturation across repeated switches; (5) command tracking recovers promptly post-switch (vx/vy/yaw error back in-band within the same bound as hard1's steady-state). FAIL on any bar; name it. Matched-parent (hard1) control required for the fixed-command panel.

**verdict**: FAIL on pre-registered gate clause (2) — the ~20M full-stress-mix hardening did NOT eliminate the direction-switch tangle/stall it was built to fix. Standard harness eval under the runs OWN trained stress_mix cfg (120s episodes, all six schedule families, instant switches; 24 total episodes across DR-0 gate + own-DR-0.35 passes): 3/24 episodes ended in a hard over_current safety termination (gate/sto ep2 fwd=0.32m slip=4.76/m; gate/sto ep3 fwd=0.27m slip=4.66/m; owncfg/sto ep3 fwd=0.77m slip=2.44/m) -- a servo jammed against a limit long enough to trip the safety cutoff, exactly the yaw-limit-saturation stall this arm targeted. Separately, gate/det ep1 is a severe leg-tangle: gait_valid=False, 3 legs sacrificed [0,2,4], slip 32/m, progress_ratio 0.03, forward 0.17m over the full 120s (video confirms near-stationary struggling, not a clean walk). No clean rise/fall boundary -- video for the 3 over_current episodes shows the robot staying upright (not a tip-over), so these are jam/stall failures, not roll falls; the harness roll_class=fell tag is driven purely by term=True per eval_checkpoint.py, not by roll angle -- worth noting for anyone reading the raw json. Base gait quality partially retained (gait_valid 6/6 on the other 23 episodes, roll clean/recovered on all but the leaning det/4, slip 1.3-2.4/m on the healthy episodes) but the core hypothesis (best-ckpt retention guard + longer full-mix exposure would cure the tangle) is REFUTED -- confirming this runs own pre-registered prediction-if-false: staged-dwell exposure is needed, not more full-mix steps from step 0. Did not additionally run the dedicated probe_dirswitch_tangle script or a legacy fixed-command matched-parent panel (clause 1) since clause (2) alone already fails the gate decisively (FAIL on any bar, per the gate text) -- no further compute spent chasing a result already settled. Next lever (queued this cycle): a staged command-difficulty curriculum via the existing tested sched.* engine driving goal.walk_cmd_stage 0->2 over training (built 08-17 for exactly this failure class, unused on this lineage until now).

