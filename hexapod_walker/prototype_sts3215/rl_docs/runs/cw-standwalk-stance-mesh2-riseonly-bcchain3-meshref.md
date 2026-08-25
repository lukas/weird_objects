# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-25T16:36:40+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-slowchain

**wandb_id**: 5wdood22

**hypothesis**: Can the robot learn to stand up from its belly when the demonstration it copies is one its own motors can actually afford? Every prior rise arm supervised toward rise_ref_belly2plant.npz, EXTRACTED from the 2.104kg primitive-family champion rollout: its mid-rise press is asymmetric (two legs folded at femur ~-57deg / tibia ~+100deg) and paced above the ~31deg/s servo velocity limit, and on the 3.5kg mesh model policies grind into that posture at the 2.64A ceiling (rung-8 evidence: pace dose bracketed non-monotonic, budget null at every pace, anchor dose 6.0 regression). Rung-9 swaps in rise_ref_mesh_scripted.npz -- minted this cycle by make_rise_ref_scripted.py from geometry alone (tuck feet to the plant footprint while the belly carries the mass, then a symmetric quasi-static Cartesian press via the trusted FK/IK) and PROVEN feasible open-loop on the mesh model itself: +83mm rise, end 0.8deg RMS from plant, cur_p95 0.53A, held-out-seed robustness PASS. Exact slowchain recipe otherwise (half-pace chain lookahead 0.25s / min_h 8mm, anchor coef 3.0, from scratch). Prediction-if-true: deep-start (flat/bridge/rsi) episodes stop pinning 2.64A -- the anchor now points at 0.53A-feasible postures -- and det valid_plant exceeds slowchain's 3/6. Prediction-if-false: deep starts still pinned/stalled with anchor loss converged -- reference content was never the blocker; the remaining suspect is reward/termination pricing.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition or close a behavior/reward class at this checkpoint. 2M canary, DR-0 rise det+sto n=6+6 (same harness as slowchain/anchordose6). PASS: det>=4/6 AND sto>=4/6 valid_plant with cur_p95<=1.5A in valid episodes and zero over_current -> promote this recipe to 8M acquisition. PARTIAL: deep-start det valid_plant > slowchain's 3/6 OR deep-start cur_p95 median < slowchain's 1.85A OR over_current terms < 3/12, even short of the bar -> mesh-native ref is a real lever; fund 8M acquisition (and consider re-dosing chain pace against the new ref). FAIL: deep starts still pinned ~2.64A or frozen (height_err_end_mm 70-90 at near-zero current; always cross-check height_err_end_mm, current/terms alone are gameable) with bc_anchor_loss_rise converged -> reference content refuted alongside pace/budget/dose; next suspect is reward/termination pricing (root-cause chain required before any patch). Judge the s0+s1 pair together as a pass-rate; single-seed counts on this rung are noisy.

**verdict**: CANARY PASS (mechanism-health; judged jointly with -s1 per pre-registered pair rule; behavioral score = PARTIAL-strong, funds acquisition). The mesh-native scripted rise reference is a REAL LEVER: DR-0 rise gate det 5/6 + sto 4/6 valid_plant vs parent slowchain 3/6+2/6; valid-ep cur_p95 median 1.36A < slowchain's 1.85A; deep starts (bridge/rsi) rise cleanly on video (sprawl->six-feet plant->full stand, tilt<2deg, h_err_end 3-9mm). Missed the strict PASS bar only on zero-over_current (3/12 terms: flat-prone det/0 + 2 rsi-sto) and 2 valid eps >1.5A p95 (2.25/1.64A, both bridge deep-press). Failure signature changed qualitatively: slowchain froze at h_err 76-79mm @0.3A; meshref failures over_current MID-RISE at h_err 16-38mm — video shows a splayed front leg that never tucks under the body, pressing the extended lever arm to the 2.64A pin. Training-reward quarter swings (12->-382) match parent slowchain's shape (recipe-normal penalty accounting), not collapse. Next per gate: promote this exact recipe to 8M acquisition as a seed pass-rate grid.

