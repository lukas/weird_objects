# cw-walkcurr-pf-fwd6-rnd10-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-23T22:46:52+00:00

**pod**: hexapod-mjx-train-2

**steps**: 4000000

**parent**: cw-walkcurr-pf-fwd6-rnd10

**wandb_id**: i5d3p177

**hypothesis**: Plain English: rnd10 (coef 0.10) never crossed the freeprog zero line in 2M but was still trending toward it (-0.101->-0.0054, monotonic) with a healthy optimizer the whole way -- unlike the 9 flat-frozen arms, this one didn't plateau, so more budget (not more dose) is the untried axis. Straight +4M continuation, same recipe, same rnd-coef, byte-identical cfg otherwise. Prediction-if-true: walk_freeprog_score crosses 0 within the extension and det gait_valid panel starts showing real forward travel. Prediction-if-false (freeprog plateaus below 0, e.g. flattens in [-0.02,0.00] without crossing, OR walk_speed keeps decaying toward exactly 0): the trajectory was asymptoting at a near-frozen state, not genuinely approaching a walking optimum -- closes the budget axis alongside the dose axis (rnd02/rnd10/rnd100), leaving BC-kickstart as the track's only unexplored escalation item.

**gate**: Same rung-1 gate: prog_ratio>0, gait_valid>=4/6 det with visible forward travel on video, walk_freeprog_score crosses 0 and stays positive, clip_fraction stays healthy. PASS = rung-1 lands (RND + budget mechanism). FAIL closes the budget axis; combined with rnd100's dose-bracket read, this fully closes RND-as-a-class before BC-kickstart.

