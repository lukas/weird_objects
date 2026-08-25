# cw-standwalk-stance-mesh2-riseonly-bcchain3-slowchain

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T14:26:05+00:00

**pod**: hexapod-mjx-train-2

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-stdanneal

**wandb_id**: 8abary62

**hypothesis**: Is the BC-anchor chain's demanded pace what forces the hot splayed press-up from deep starts? The stdanneal parent tracks the state-aligned rise chain tightly (anchor_loss 0.05) yet flat/rsi starts saturate current at 2.64A mid-rise: one candidate cause is that lookahead 0.5s + min_h_ahead 15mm demands a lift rate whose torque exceeds the current budget on the 3.5kg mesh body (the ref was extracted from the 2.1kg 25Hz primitive). This arm reruns the exact stdanneal recipe from scratch with the chain pace halved (lookahead_s 0.5->0.25, min_h_ahead_mm 15->8), giving the policy a nearer, gentler anchor target so it can rise on a slower, cooler trajectory. Prediction-if-true: deep-start cur_p95 unpins from 2.64A and over_current terms drop even where plants are not yet valid. Prediction-if-false: deep starts equally pinned at half pace -- pace is not the driver, the press-up POSTURE (splayed lever arms) is baked into the ref's flat segment, confirming the mesh-native-ref lever jointly with cont8.

**gate**: 8M acquisition, recipe = stdanneal parent with chain pace halved. Rise DR-0 det+sto n=6+6 AND own-DR(0.2) det+sto n=6+6. PASS: det >=4/6 AND sto >=4/6 valid_plant, cur_p95<=1.5A in valid episodes, zero over_current terms. PARTIAL: deep-start (flat/bridge/rsi) cur_p95 unpinned (<2.4A median) or over_current terms < parent's 8/12 DR-0, even if valid counts similar -- pace matters, dose the pace next. FAIL: deep starts still pinned ~2.64A with anchor tracking converged -- pace refuted; with cont8 also failed, mint a mesh-native rise ref from scripted IK as the rung-9 lever.

