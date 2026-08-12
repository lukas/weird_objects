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

**verdict**: FAIL per own gate but mechanism-CONFIRMING: the height-floor pursuit (floor=15mm) moved the det flat-rise stall from ~100mm short (footlow1) to 15-16mm short and fixed sto flat outright (sto rise 6/6 incl 4/4 flat vs footlow1's 2/6; det rise 4/6, flat 0/2, roll_tail 4-4.8deg leaning, leg idx3 duty 0.01). Lower retained 12/12 (sub-mm to few-mm clearances). But the pre-registered seesaw alternative FIRED: det hold reopened the idx1 park (duty 0.03 in all 6 eps; valid_plant still 6/6) despite bc_anchor_foot_z=1 on. Gate-required residual-stall stat is ambiguous: probe_anchor_align at seed 0 det flat shows NO stall (h_err 3mm, anchor target = path-end plant j=313, mse 0.003) while the eval's seeded det flat episodes end 15mm short = seed-dependent endgame, needs the seeded audit before the next arm. DIG-IN flagged.

