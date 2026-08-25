# cw-standwalk-stance-mesh2-riseonly-bcchain3-slowchain

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PARTIAL

**created**: 2026-08-25T14:26:05+00:00

**pod**: hexapod-mjx-train-2

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-stdanneal

**wandb_id**: 8abary62

**hypothesis**: Is the BC-anchor chain's demanded pace what forces the hot splayed press-up from deep starts? The stdanneal parent tracks the state-aligned rise chain tightly (anchor_loss 0.05) yet flat/rsi starts saturate current at 2.64A mid-rise: one candidate cause is that lookahead 0.5s + min_h_ahead 15mm demands a lift rate whose torque exceeds the current budget on the 3.5kg mesh body (the ref was extracted from the 2.1kg 25Hz primitive). This arm reruns the exact stdanneal recipe from scratch with the chain pace halved (lookahead_s 0.5->0.25, min_h_ahead_mm 15->8), giving the policy a nearer, gentler anchor target so it can rise on a slower, cooler trajectory. Prediction-if-true: deep-start cur_p95 unpins from 2.64A and over_current terms drop even where plants are not yet valid. Prediction-if-false: deep starts equally pinned at half pace -- pace is not the driver, the press-up POSTURE (splayed lever arms) is baked into the ref's flat segment, confirming the mesh-native-ref lever jointly with cont8.

**gate**: 8M acquisition, recipe = stdanneal parent with chain pace halved. Rise DR-0 det+sto n=6+6 AND own-DR(0.2) det+sto n=6+6. PASS: det >=4/6 AND sto >=4/6 valid_plant, cur_p95<=1.5A in valid episodes, zero over_current terms. PARTIAL: deep-start (flat/bridge/rsi) cur_p95 unpinned (<2.4A median) or over_current terms < parent's 8/12 DR-0, even if valid counts similar -- pace matters, dose the pace next. FAIL: deep starts still pinned ~2.64A with anchor tracking converged -- pace refuted; with cont8 also failed, mint a mesh-native rise ref from scripted IK as the rung-9 lever.

**verdict**: PARTIAL - real progress, pace lever CONFIRMED (pre-registered prediction-if-true). Halving the BC-anchor chain pace from-scratch (lookahead 0.5s->0.25s, min_h_ahead 15mm->8mm, otherwise identical stdanneal recipe, 8M budget) measurably unpins the deep-start (flat/bridge/rsi) press-up vs the stdanneal parent: gate deep-start cur_p95 MEDIAN falls 2.64A -> 1.85A, gate over_current terms fall parent's 8/12 -> 3/12, and DR-0 det valid_plant improves 2/6->3/6 (sto flat at 2/6). Still short of the registered PASS bar (need det>=4/6 AND sto>=4/6 AND cur_p95<=1.5A in ALL valid eps): the valid deep-start plants that DO occur (bridge det 2.09/1.85A, rsi sto in own-DR not yet checked) are still >1.5A, so 'unpinned' means 'below the 2.64A ceiling', not yet 'cool'. bc_anchor_loss_rise is higher than the parent's plateau (0.092 vs 0.05) -- the gentler target is tracked less tightly, which is expected and not disqualifying (footz_loss also higher, 3.64 vs parent territory). This REFUTES the run's own prediction-if-false ('deep starts equally pinned at half pace') and refutes the joint-FAIL branch that would have forced minting a mesh-native IK rise ref -- since cont8 (more budget, same pace) and reanneal (more noise, same pace) both independently FAILED to move the deep-start metric (see their verdicts), pace is the one lever of the three that worked. Next: dose the pace lever further (quarter/eighth of the original), not budget or noise.

