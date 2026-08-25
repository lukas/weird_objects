# cw-standwalk-stance-mesh2-holdprod-f01-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T08:52:44+00:00

**pod**: hexapod-mjx-train-3

**steps**: 6000000

**parent**: cw-standwalk-stance-mesh2-holdload1min

**wandb_id**: n02jq4xp

**hypothesis**: Seed twin (seed 1) of holdprod-f01: Can the robot keep standing on all six feet if every foot that RETURNS to load immediately earns more -- i.e. does per-foot graded income (product-over-feet load gate with a LOW floor) fix the min-variant's zero cross-foot gradient? All 3 load-min arms (holdload1min seed0 rear-up OC lock, -s1 belly-flop freeze, -warm tilt collapse) failed in DIFFERENT basins despite STARTING at the paid plant pose: the min gate pays flat 0.1x scraps the moment ANY foot unloads, so a noise excursion off the plant has no graded income path back. The product path pays per-foot: each foot re-loaded multiplies income back toward 1.0. The product's historical defeat (crouchrise trio) used floor 0.5 (one shed foot = half pay, sufficient-and-cheaper); floor 0.1 makes one shed foot scraps like min does while keeping the gradient. Probe logs/probe_stance_pricing_loadfloor.json: honest six-foot hold unchanged 1471.6/ep; the measured holdonly1 stilt prices to -161.0 (f0.1) / -143.4 (f0.3), HARDER tax than min's -94.5 -- orderings green. Prediction-if-true: policy stays in/returns to the plant basin, six-foot quiet hold by 6M. Prediction-if-false: same knock-off destabilization -> gradient shape was not the missing piece; the sibling -dr0/-ent4 (noise) arms decide the axis. Strongest alternative cheat: five-foot stance with the sixth riding the s_i on-ramp -- watch hold_load_factor + duty.

**gate**: Hold panel at 6M: pod_eval hold DR-0 det+sto n=6+6. PASS: >=10/12 survive 15s with zero over_current/tilt terminations AND det episodes show six-foot stance (valid_plant true, or all-leg duty>=0.9 with end_clear<5mm on all legs) AND cur_p95<=1.0A. FAIL: still sheds/rears/freezes -- the product-gradient lever is closed at both floors; defer to the -dr0/-ent4 noise axis and pre-register any combo arm before launching.

**verdict**: Result: FAIL -- seed 1 under the product/floor-0.1 gate finds the cold BELLY-REST basin: video shows it start planted, then fold all six legs UP and park belly-down for the whole episode. Evidence: DR-0 gate 0/12 ok with ZERO terminations (survives 15s), all-leg duty 0.01-0.14, end_clear ~40mm on all six feet, cur_p95 0.52-0.54A, valid_plant 0/12, ret ~-777/-918 vs honest +1472; same basin as the min-variant s1 twin. Why: 'do nothing safely on the belly' dominates when the paid plant+still income is unreachable through exploration noise (policy std ~1.05 makes planted stillness unearnable during rollouts) -- pricing/gradient shape can't fix that. See f01 verdict for the 4-arm grid synthesis (basin splits by seed, not floor). Next: noise axis (-dr0/-ent4 siblings) decides; combo arms pre-registered only after those read.

