# cw-standwalk-stance-mesh2-holdload1min-dr0

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T08:46:16+00:00

**pod**: hexapod-mjx-train-1

**steps**: 6000000

**parent**: cw-standwalk-stance-mesh2-holdload1min-s1

**wandb_id**: xw87z926

**hypothesis**: Plain English: does the robot learn to just stand on all six feet once the simulator stops randomizing the physics during training? Single lever off the holdload1min scratch recipe: dr-scale 0.2 -> 0.0. Rationale: s1 showed the reward is ALIGNED (honest plant +1471/ep vs its belly-freeze -802) and hold episodes START inside the paying plant basin, yet PPO walks out of it -- under flat DR 0.2 from step 0 the scripted plant pose is not an equilibrium in most worlds, so a random-init policy is knocked over immediately, learns falls are expensive, and retreats to the belly-freeze attractor. At DR-0 the plant start IS a true equilibrium (probe's honest scripted hold banks 1471 at DR-0), so quiet standing should be discoverable by pure local exploration. Prediction-if-true: six-foot hold with income within 6M; then the next rung builds a DR ramp 0->0.2 to buy robustness back (rung-2 candidate (b), still unbuilt). Prediction-if-false (belly-freeze even at DR-0): DR is exonerated and the blocker is optimization/exploration itself -- read jointly with the ent4 sibling.

**gate**: Hold panel at 6M: pod_eval hold DR-0 det+sto n=6+6 (own-DR == DR-0 for this arm), >=10/12 survive with zero OC/tilt terms, six-foot stance (valid_plant or all-leg duty>=0.9), cur_p95<=1.0A.

**verdict**: Result: FAIL -- flat DR-from-step-0 does not rescue the load-min income gate; it trades one failure basin for a worse one. Evidence: 12/12 hold episodes (det+sto) terminate tilt_roll (not the s1 belly-flop-freeze or the warm-stilt tilt-term), video shows the robot start correctly planted then progressively roll onto its side and topple by ~10-15s; training reward is monotonically WORSENING each quarter (-111 -> -355 -> -651 -> -718), a genuine FAIL under the 08-21 ruling (reward declining + eval bad), not a continuation case. Why: DR-scale was never the reason PPO couldn't find the six-foot-plant gradient under load-min pricing -- with DR off the random-init policy still can't discover the plant basin, it just falls a different way (rolling instead of freezing or ceiling-riding). This is now the 3rd distinct failure basin across the load-min-gate rung (seed0 OC-ceiling rear-up, s1 belly-flop-freeze, this tilt_roll) plus the warm-stilt tilt-term -- 4/4 arms fail, each differently, which reads as a genuinely zero-gradient reward shelf rather than an optimization-lever problem (dr-scale, warm-start) fixable by tweaking training mechanics. Next: read jointly with the sibling -ent4 (entropy-retune) arm when it finishes; if that also fails, the load-min income-gate formulation itself (not the optimizer around it) is the lever to redesign -- e.g. a shaped/graded per-foot load-error term instead of a hard min-over-feet gate, or an explicit curriculum that starts from the honest six-foot plant pose (RSI-style) rather than random init.

