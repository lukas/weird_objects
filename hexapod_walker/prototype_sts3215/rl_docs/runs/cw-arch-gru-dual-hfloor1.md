# cw-arch-gru-dual-hfloor1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-12T18:46:46+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-arch-gru-dual1

**wandb_id**: xzgepu11

**hardware_ready**: False

**hypothesis**: Plain English: give the standing-up teacher inside the new two-brain GRU a rule that it must always aim at least 15mm higher than the robot currently is, so it can never get stuck copying a demo that barely moves. cw-arch-gru-dual1 (10M) already proved the two-core architecture fixes the walk freeze (det walk gait_valid 6/6, real translation, hold/lower both 6/6 and cleaner drag/roll than the old shared-trunk parent) but its rise is one episode short of the gate (det 1/6, needs >=2/6) -- bridge 0/3, crouch 1/1, flat 0/2. The exact same failure shape (state-aligned BC-anchor pursuit, 0.5s lookahead) was root-caused on the MLP stance lineage as a PLATEAU FIXED POINT: the matched demo frame pins where the recorded reference itself crawls only 0-25mm over 5+ seconds, so the pursuit target commands too little height gain and loaded-servo sag cancels it -- and train.bc_anchor_min_h_ahead_mm (already landed, tested, in-repo) fixed it there (footlow1 -> footlow2, det rise 3/6 -> 12/12). This arm adds ONLY that one switch (=15mm) to the identical dual1 recipe, warm-started from dual1's own checkpoint. Prediction-if-true: det rise recovers toward >=2/6 with a real flat or bridge success (not just crouch), while walk (gait_valid>=5/6, prog_ratio>=0.80) and hold/lower (>=4/6 each) hold at dual1's level. Prediction-if-false: rise stays stuck near 1/6 even with the floor active -- meaning the dual-core rise miss is DATA-poverty inherited from the BC-distill (same mechanism as the ft1/ft2 MLP-style finetunes, which never had rise demos to begin with), not a plateau, and the real fix is the operator's in-progress DAgger rise redistillation, not this lever.

**gate**: 2M discovery, det+sto @DR0 gate cfg (same gate cfg as dual1): PASS if det rise >=2/6 with >=1 non-flat-start success (bridge or flat, not just crouch) AND det walk gait_valid >=5/6 with prog_ratio med >=0.80 (freeze fingerprint stays absent) AND hold det >=4/6 AND lower det >=4/6. FAIL if rise stays <=1/6 despite the floor (plateau lever doesn't transfer to this arch -> data-poverty is the real blocker, escalate to the DAgger-redistill lever, no further coefficient/floor-mm variant on this arm) OR walk/hold/lower regress below dual1's own numbers (the floor pursuit fights the other anchors -> a new seesaw, split the levers).

**verdict**: FAIL: the min_h_ahead floor lever (the exact fix that took the MLP stance lineage's rise 3/6->12/12) does NOT transfer to the dual-core GRU -- on a fair larger-sample recheck (n=12/seed=1, same method used to correct dual1's own noisy draw) rise is 5/12 det (crouch 5/5, bridge 0/4, flat 0/3, ZERO non-crouch wins) and 1/12 sto, both WORSE than dual1's identical-method 7/12 det / 4/12 sto (which had 2 real non-crouch wins). New failure mode: 3-4 of the non-crouch episodes now trip over_current (0 in dual1's matched draw) -- video-confirmed as an honest stall (robot sits low, splayed, motionless) running out of current headroom over the full 15s, not a thrash or new cheat. Walk/hold/lower all hold clean: det walk gait_valid 6/6, zero sacrificed legs, prog_ratio med 0.99 (real translation, video-confirmed); hold det 6/6 valid_plant, duty 0.99-1.0 all feet, drag 48mm (vs dual1 55mm, slightly better); lower det 6/6 valid_plant, duty ~1.0, drag ~90mm (vs dual1 ~99mm, similar) -- the dual-core walk-freeze fix and stance quality both survive intact. One cost to flag: own-DR0.5 lower/sto picked up 2 new tilt_roll falls (4/6 vs dual1's clean 6/6). Per pre-registration this is the false branch decisively: the plateau-fixed-point lever is architecture-specific to the MLP lineage, not a general anchor-shape fix; rise's remaining gap here is data-poverty in the BC-distill, not a supervision-target problem. No further coefficient/floor-mm variant on this arm -- escalate to the operator's in-progress DAgger rise-redistillation.

