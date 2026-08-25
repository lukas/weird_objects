# cw-standwalk-stance-mesh2-holdload1min-dr0

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-25T08:43:15+00:00

**pod**: hexapod-mjx-train-1

**steps**: 6000000

**parent**: cw-standwalk-stance-mesh2-holdload1min-s1

**hypothesis**: Plain English: does the robot learn to just stand on all six feet once the simulator stops randomizing the physics during training? Single lever off the holdload1min scratch recipe: dr-scale 0.2 -> 0.0. Rationale: s1 showed the reward is ALIGNED (honest plant +1471/ep vs its belly-freeze -802) and hold episodes START inside the paying plant basin, yet PPO walks out of it -- under flat DR 0.2 from step 0 the scripted plant pose is not an equilibrium in most worlds, so a random-init policy is knocked over immediately, learns falls are expensive, and retreats to the belly-freeze attractor. At DR-0 the plant start IS a true equilibrium (probe's honest scripted hold banks 1471 at DR-0), so quiet standing should be discoverable by pure local exploration. Prediction-if-true: six-foot hold with income within 6M; then the next rung builds a DR ramp 0->0.2 to buy robustness back (rung-2 candidate (b), still unbuilt). Prediction-if-false (belly-freeze even at DR-0): DR is exonerated and the blocker is optimization/exploration itself -- read jointly with the ent4 sibling.

**gate**: Hold panel at 6M: pod_eval hold DR-0 det+sto n=6+6 (own-DR == DR-0 for this arm), >=10/12 survive with zero OC/tilt terms, six-foot stance (valid_plant or all-leg duty>=0.9), cur_p95<=1.0A.

**refused_reason**: discovery runs cap at 2000000 steps (asked 6000000): the question is 'did qualitatively correct behavior emerge?' - continue as --phase hardening with --evidence.

