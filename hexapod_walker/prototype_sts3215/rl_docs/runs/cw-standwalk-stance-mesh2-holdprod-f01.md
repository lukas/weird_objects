# cw-standwalk-stance-mesh2-holdprod-f01

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T08:54:52+00:00

**pod**: hexapod-mjx-train-2

**steps**: 6000000

**parent**: cw-standwalk-stance-mesh2-holdload1min

**wandb_id**: c5i5ktyj

**hypothesis**: Can the robot keep standing on all six feet if every foot that RETURNS to load immediately earns more -- i.e. does per-foot graded income (product-over-feet load gate with a LOW floor) fix the min-variant's zero cross-foot gradient? All 3 load-min arms (holdload1min seed0 rear-up OC lock, -s1 belly-flop freeze, -warm tilt collapse) failed in DIFFERENT basins despite STARTING at the paid plant pose: the min gate pays flat 0.1x scraps the moment ANY foot unloads, so a noise excursion off the plant has no graded income path back. The product path pays per-foot: each foot re-loaded multiplies income back toward 1.0. The product's historical defeat (crouchrise trio) used floor 0.5 (one shed foot = half pay, sufficient-and-cheaper); floor 0.1 makes one shed foot scraps like min does while keeping the gradient. Probe logs/probe_stance_pricing_loadfloor.json: honest six-foot hold unchanged 1471.6/ep; the measured holdonly1 stilt prices to -161.0 (f0.1) / -143.4 (f0.3), HARDER tax than min's -94.5 -- orderings green. Prediction-if-true: policy stays in/returns to the plant basin, six-foot quiet hold by 6M. Prediction-if-false: same knock-off destabilization -> gradient shape was not the missing piece; the sibling -dr0/-ent4 (noise) arms decide the axis. Strongest alternative cheat: five-foot stance with the sixth riding the s_i on-ramp -- watch hold_load_factor + duty.

**gate**: Hold panel at 6M: pod_eval hold DR-0 det+sto n=6+6. PASS: >=10/12 survive 15s with zero over_current/tilt terminations AND det episodes show six-foot stance (valid_plant true, or all-leg duty>=0.9 with end_clear<5mm on all legs) AND cur_p95<=1.0A. FAIL: still sheds/rears/freezes -- the product-gradient lever is closed at both floors; defer to the -dr0/-ent4 noise axis and pre-register any combo arm before launching.

**verdict**: Result: FAIL -- per-foot graded income (product gate, floor 0.1) does NOT rescue seed 0; it reproduces holdload1min's rear-up/OC-lock basin almost exactly. Evidence: DR-0 gate 0/12 (12/12 over_current, det duty [0.04,1.0,0.62,0.29,0.44,0.44], 3-4 feet aloft, valid_plant 0/12, sto cur_p95 up to 2.63A); reward trajectory near-identical to the min-variant twin (-103/-408/-646/-764 vs -102/-406/-626/-691). Why: gradient shape between feet was not the binding constraint for this seed -- the policy leaves the paid plant basin early and fights gravity at the current ceiling regardless of how partial recovery is priced. Grid read (4 arms, 2 floors x 2 seeds, all 0/48): failure basin splits by SEED not by floor (seed0=OC rear-up, seed1=cold belly-rest), so income micro-structure is exonerated; the live axis is exploration noise vs the still/plant requirements (sibling -dr0/-ent4 arms). What's next (pre-registered): product-gradient lever CLOSED at both floors; defer to -dr0/-ent4; any combo arm gets pre-registered first.

