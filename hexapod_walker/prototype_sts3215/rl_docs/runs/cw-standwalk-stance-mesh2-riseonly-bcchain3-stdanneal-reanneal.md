# cw-standwalk-stance-mesh2-riseonly-bcchain3-stdanneal-reanneal

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T14:30:30+00:00

**pod**: hexapod-mjx-train-1

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-stdanneal

**wandb_id**: lqav84km

**hypothesis**: Can a second round of noise annealing shake the rise policy out of its hot splayed press-up basin? The stdanneal parent perfected the crouch-path rise (6/6 valid at 0.9A) but from flat/rsi starts it is stuck in a torque-saturated splay-press (cur pinned 2.64A) that low-noise refinement may only polish, never escape. This arm warm-starts from the parent checkpoint and RE-RUNS the full log-std 0->-4 anneal (simulated-annealing round 2): the re-injected exploration noise lets PPO sample tuck-first/cooler rise trajectories from deep starts, then the anneal locks in whichever is best -- the same lever that took hold sto 0/6->6/6 and cooled 2.64A->0.5A. Prediction-if-true: flat/rsi episodes shift from press-up to a cooler strategy, det+sto >=4/6 valid. Prediction-if-false: re-annealing degrades the crouch-path competence without unpinning deep starts -- noise re-injection is destructive on a converged fine skill, and the basin escape needs a better ref, not noise.

**gate**: 8M re-anneal continuation. Rise DR-0 det+sto n=6+6 AND own-DR(0.2) det+sto n=6+6. PASS: det >=4/6 AND sto >=4/6 valid_plant at DR-0, cur_p95<=1.5A in valid episodes, zero over_current terms; flat/rsi-start videos show a rise without the pinned press-up. PARTIAL: deep-start (flat/bridge/rsi) valid count strictly above parent's 3/13 with crouch-path competence intact (crouch starts still all valid). FAIL: crouch-start competence lost (any crouch episode invalid) OR deep starts unchanged at 2.64A pin -- re-anneal is the wrong lever; fold into the cont8/slowchain read.

**verdict**: FAIL - MECHANISM (pre-registered branch). Fresh 0->-4 noise re-injection from the stdanneal ckpt (re-anneal, same chain pace lookahead 0.5s/min_h 15mm) does NOT unpin the deep-start press-up either. DR-0 gate det 1/6 (WORSE than parent's 2/6) + sto 4/6 valid_plant (better than parent's 2/6) -- noise dithers a couple of rsi starts into a lucky valid plant in stochastic mode (2 rsi sto successes at cur_p95 2.09/2.27) but det mode got worse. Deep-start cur_p95 median stays 2.64A (pinned, same as parent/cont8) and bc_anchor_loss_rise is 0.0512, matching the plateau -- re-exploring the noise basin did not find a cooler rise trajectory. over_current terms 7/12 vs parent's 8/12 (same marginal improvement as cont8, i.e. no differential benefit from re-annealing vs just continuing flat). Reward final-quarter +6.8 (mild recovery from parent's -48) but again buys crouch/rsi-lucky refinement, not deep-start unpinning. Confirms cont8's read: noise dose was never the deep-start blocker (the hold-rung precedent for re-anneal helping doesn't transfer here); slowchain's structural pace-halving is the lever that actually moved the metric. Next: dose the pace lever further, not noise.

