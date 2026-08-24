# cw-walkcurr-pf-fwd6-rnd10-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T22:46:52+00:00

**pod**: hexapod-mjx-train-2

**steps**: 4000000

**parent**: cw-walkcurr-pf-fwd6-rnd10

**wandb_id**: i5d3p177

**hypothesis**: Plain English: rnd10 (coef 0.10) never crossed the freeprog zero line in 2M but was still trending toward it (-0.101->-0.0054, monotonic) with a healthy optimizer the whole way -- unlike the 9 flat-frozen arms, this one didn't plateau, so more budget (not more dose) is the untried axis. Straight +4M continuation, same recipe, same rnd-coef, byte-identical cfg otherwise. Prediction-if-true: walk_freeprog_score crosses 0 within the extension and det gait_valid panel starts showing real forward travel. Prediction-if-false (freeprog plateaus below 0, e.g. flattens in [-0.02,0.00] without crossing, OR walk_speed keeps decaying toward exactly 0): the trajectory was asymptoting at a near-frozen state, not genuinely approaching a walking optimum -- closes the budget axis alongside the dose axis (rnd02/rnd10/rnd100), leaving BC-kickstart as the track's only unexplored escalation item.

**gate**: Same rung-1 gate: prog_ratio>0, gait_valid>=4/6 det with visible forward travel on video, walk_freeprog_score crosses 0 and stays positive, clip_fraction stays healthy. PASS = rung-1 lands (RND + budget mechanism). FAIL closes the budget axis; combined with rnd100's dose-bracket read, this fully closes RND-as-a-class before BC-kickstart.

**verdict**: RND-coef=0.10 +4M budget continuation FAILS the rung-1 gate and closes the budget axis (paired with rnd100's already-closed dose axis -> RND-as-a-class fully refuted at any bank-safe dose or budget before BC-kickstart). Reward quarters rose then regressed (59.9/63.1/51.4/37.9 -- peaked at 25% through, worsened over the back half), confirming this was not simply undertrained. Two distinct behaviors on the SAME checkpoint: the SB3 --best-ckpt selection (picks on accumulated episodic reward) reverts to the identical belly-sit signature seen everywhere else on this track (height_err_end_mm=116.3, all 6 det legs sacrificed, speed 0.005 m/s); the FINAL checkpoint instead shows a genuinely new failure mode never seen before on this track -- a forward lurch with real forward_dist (~0.056m) and gait_valid=True on all 6 det episodes, but terminates tilt_pitch 6/6 det (and several sto) every single time. Video (contact sheet + full frame strip, walk_det_0.mp4) shows legs splaying and the body pitching down/forward within ~1s -- a controlled fall, not alternating stance/swing cycling. Mechanism confirmed: because a fall forfeits the rest of the episode's income and eats term_penalty while belly-sit collects a small steady income for the full 25s, the reward is pricing 'survive motionless' strictly above 'attempt to walk, risk a fall' -- itself reward-shape evidence (not a fresh exploration/budget finding), independently corroborating the belly-sit root-cause analysis that already motivated the height-gate mechanism (hgt1/hgt2, in flight).

