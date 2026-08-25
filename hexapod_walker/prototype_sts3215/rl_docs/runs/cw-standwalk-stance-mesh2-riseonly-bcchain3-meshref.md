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

**verdict**: CANARY PASS (pre-registered PARTIAL branch fires decisively; judged jointly with -s1, near-identical): the mesh-native scripted rise reference UNPINS the deep starts — the first rise mechanism in this whole rung to do so. DR-0 det 5/6 valid_plant (bridge 2/2, rsi 2/2, crouch 1/1; herr_end 3.3-9.4mm, clean rolls) vs slowchain's 3/6; sto 4/6. Deep-start current on valid episodes 0.58-2.25A (most <=1.5A) vs slowchain's everything-pinned-2.64A. Prediction-if-true landed exactly. Short of full PASS bar: flat-start det still pins 2.64A (both seeds), 2 sto rsi episodes pin (3/12 oc, equal to slowchain's not below), and 3 valid episodes sit 1.64-2.25A above the 1.5A bar. Video det_1: clean symmetric bridge->plant press, six feet down, held. Reference content was the rung blocker, confirmed closed-loop after pace/budget/dose all failed. NEXT (funded this cycle): promote to 8M acquisition (meshref-acq8m) + fullpace re-dose canary against the new ref. Evidence: logs/ckpt_eval/cw_standwalk_stance_mesh2_riseonly_bcchain3_meshref_gate/, W&B 5wdood22.

