# cw-dep-fresh1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T04:33:37+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**wandb_id**: qmtctc9g

**hardware_ready**: False

**hypothesis**: OPERATOR PRIORITY (hardware session 08-09 night, RL_LOG session 3). Fresh-init deployment-contract arm, the rocking-gait question: the scripted tripod gait that ACTUALLY WALKS the robot rocks +-10-20deg roll/pitch, which our 10deg tilt termination forbids -- our RL line may be locked into low-amplitude creep because real weight transfer is a termination event. Hypothesis: from scratch under the deployed contract (meas:=ref velocity obs via goal.walk_obs_body_vel=2, 25deg tilt envelope, field-standard exploration log_std 0 / ent 0.005 per BEST_PRACTICES_AUDIT) a weight-transfer gait emerges that walks with body rock like the scripted gait instead of creeping. If FALSE (converges to the same creep or fails gait_valid), rocking-permission alone is not the creep bottleneck.

**gate**: own-cfg det+sto 6/6 @15s gait_valid 12/12, 0 term; video frames watched SPECIFICALLY for weight transfer vs creep + roll amplitude report vs the +-10-20deg hardware envelope; slip/m reported (feet MAY slide per hardware finding f)

**verdict**: PASS on numeric gate: own-cfg (contract-exact obs, DR0.2, 25deg envelope) det+sto 6/6 gait_valid 12/12, 0 falls; det prog med 1.28 slip/m med 1.50 fwd 1.02m, sto prog med 0.81 slip/m med 2.30 fwd 0.67m (one sto draw at prog -0.01 slip 21 is the known lineage fixed-draw march-in-place attractor, not new). QUALITATIVE (P0 item 1, the reason this run matters): frame-strip tilt reading across all 6 det + the flagged sto episode stays low-amplitude throughout (mostly 0-3deg roll/pitch, rare spikes to ~9-10deg) despite fresh init + 25deg permission + honest velocity obs -- does NOT produce the hoped visible weight-transfer/rocking gait; legs do genuinely lift and cycle (all six leg positions swing at different times, no flag leg, no drag/skate), current mean 0.3-0.7A (right hardware ballpark though this arm predates the k_current=0 ruling). Hypothesis item-1 (fresh init + rocking permission -> visible rocking) is REFUTED: same low-amplitude character as cw-dep-vref1-r1 (champion warm-start under the same contract, itself verdicted "low-amplitude six-leg creep gait") -- the flat gait looks like a reward-pricing/incentive property of the sim economics, not a warm-start-prior artifact. Not hardware-ready on its own; informative negative -- getting real weight-transfer will need deliberate incentive shaping, not just permission + honest obs.

