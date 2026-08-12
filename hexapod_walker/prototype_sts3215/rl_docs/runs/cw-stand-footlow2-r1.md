# cw-stand-footlow2-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-12T11:27:29+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-footlow1

**wandb_id**: 5yb50fta

**hardware_ready**: False

**hypothesis**: Plain English: make the stand-up imitation always aim at least 15mm ABOVE where the robot currently is, so it can never teach the robot to stay lying down. The footlow1 dig-in measured why flat rises stall: the recorded demo crawls 0-25mm over 5+ seconds, so the anchor's half-second-ahead target at a stalled belly state commands only 1-5mm of height gain, servo lag cancels it, the matched demo frame pins (0 ticks advance over 3s), and the policy follows that supervision perfectly (its anchor error is lowest DURING the stall). This arm is footlow1's exact recipe plus one new switch (train.bc_anchor_min_h_ahead_mm=15, the height-floor pursuit landed from that audit). Prediction-if-true: det rise recovers to >=5/6 valid_plant including flat starts, while the six-foot hold and the 12/12 lower (footlow1's twin wins) are retained. Prediction-if-false: rise still stalls ~100mm short WITH the floor active — meaning the plateau fixed point is not the whole story (e.g. the PPO reward equilibrium or the off-path bridge matching dominates), and the next audit target is the reward side at the stalled state, not the anchor. Strongest alternative: the floor unpins rise but the more aggressive rise supervision bleeds into hold/lower and reopens a park (the anchormix seesaw in a new form).

**gate**: PASS if det rise >=5/6 valid_plant with flat starts succeeding (recovering from footlow1's 3/6) AND det hold every foot duty >=0.5 in all 6 episodes (no park regression from footlow1's clean 0.94+) AND det+sto lower >=10/12 valid_plant (retaining footlow1's 12/12). FAIL if rise stays <=3/6 with the same belly stall, or hold parks any foot, or lower regresses below 10/12. Record mse(act,bc_target) at any residual stall (low = anchor still teaches it; high = PPO fights the floor).

**verdict**: FAIL per gate letter, but the DIG-IN overturns both flagged residuals and CONFIRMS the height-floor mechanism completely. OBSERVATIONS: (1) the 'seed-dependent 15mm det flat-rise endgame' is an EVAL LABELING ARTIFACT — the gate eval inherits the run's goal.rise_rsi_frac=0.5 and _start_kind() labeled RSI mid-path spawns 'flat'; the floored probe_anchor_align across seeds 0-5 (12 det flat eps) lands EVERY cold flat rise at h_err -2.6..+3.0mm (anchor at path end j=313, mse(act,tgt) 0.0028, 6/6 contacts), and an RSI-OFF rerun of the rise gate is det 6/6 valid_plant (flat incl., roll_tail <=0.3deg). The 15mm-short leaning episodes are perturbed mid-path RSI spawns only. (2) the 'reopened idx1 hold park' is a +0.9mm COMMANDED hover (FK probe vs q_nom; footlow1's same foot commands +0.4mm at duty 0.97) — the binary 0.5N duty stat flips on ~0.5mm of command; NOT the historical 10mm weight-shed park; foot_z loss at 0.9mm on a 10mm scale ~0.008 = below supervision resolution. INTERPRETATION: rise-from-flat (the last broken stance mode) is SOLVED in this checkpoint; the hold micro-interference is real but sub-mm. Eval labeling fixed in code (start_kind='rsi', snapshot da367c9). NEXT: 10M hardening cw-stand-footlow2-hard1 + one-variable foot_z_mm-sharpen discovery. Artifacts: logs/experiments/cw-stand-footlow2-r1/digin/, logs/ckpt_eval/cw_stand_footlow2_r1_rise_norsi/.

