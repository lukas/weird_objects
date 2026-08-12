# cw-stand-footlow1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-12T10:06:16+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-footz1-hard1

**wandb_id**: 8zvm5r38

**hardware_ready**: False

**hypothesis**: Plain English: teach the standing robot to plant all six feet AND sit/lower cleanly AT THE SAME TIME, by combining two BC-anchor fixes that each already solved one half of the problem alone but have never been tried together. Warm-start from footz1-hard1 (keeps its working hold fix, train.bc_anchor_foot_z=1.0) and add the four lower-anchor cfg keys that independently solved lower in cw-stand-anchormix1-r1 (bc_anchor_lower, bc_anchor_stratified, bc_anchor_state_aligned, bc_anchor_lookahead_s=0.5) -- one bundle, one mechanism (lower supervision), added on top of an already-working hold mechanism. Prediction-if-true: det+sto hold stays clean (every foot duty >=0.5, matching footz1-hard1) AND det lower recovers to >=5/6 with no new outrigger pattern (matching anchormix1-r1's 6/6) -- the two fixes are additive because they supervise different mode-ticks (hold vs lower) and stratification exists precisely to stop cross-mode dilution. Prediction-if-false: adding lower's anchor volume dilutes hold's foot-z supervision again despite stratification (a NEW dilution axis, since foot_z is a different loss term than the plain joint-MSE anchor stratification was tuned against), re-opening the hold park on a different foot -- meaning the two anchor terms need their OWN stratification split, not just per-mode weighting.

**gate**: PASS if det hold: every foot duty >=0.5 in all 6 episodes (no regression from footz1-hard1) AND hold det+sto valid_plant >=10/12 AND det lower >=5/6 valid_plant with no outrigger pattern (recovering from footz1-hard1's 0/6) AND det rise >=5/6 valid_plant with no new falls (matching footz1-hard1's own baseline). FAIL if any foot parks in hold (duty <0.5) or lower stays <5/6 or rise regresses. Record per-mode train/bc_anchor_loss_* trajectories (hold vs lower) either way.

**verdict**: FAIL per own gate (det rise 3/6, sto 2/6 — flat-start stall at 90-107mm height err, the anchormix lineage's known det flat-rise-stall signature; hold det+sto valid_plant 9/12 via 3 sto current-spec misses). INFORMATIVE: the merge is additive on hold+lower — first arm ever with clean six-foot hold (det duty >=0.94 all feet) AND 12/12 lower with sub-mm clearances (parent 0/12 at up to 126mm); the pre-registered dilution branch (hold park reopening) did NOT fire — the cost moved to rise. bc_anchor_loss_rise converged low (0.011) during the stall: anchor-blind to rise progress, same lesson class as the park. Roll/drag honest: all non-stall episodes roll_class clean, hold det slip 0.181m.

