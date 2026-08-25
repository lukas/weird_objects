# cw-standwalk-stance-mesh2-holdload1min-warm

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T08:15:00+00:00

**pod**: hexapod-mjx-train-4

**steps**: 4000000

**parent**: cw-standwalk-stance-mesh2-holdonly1

**wandb_id**: eagwo962

**hypothesis**: Does repricing rescue the ALREADY-balanced stilt? Warm-start from holdonly1's checkpoint (survives 15s but on three feet) with the load-min hold-income gate ON. holdonly1-acq1 showed continuing WITHOUT repricing destabilizes the stilt into tilt_pitch collapse (0/24); this arm tests whether the load gate instead deforms it into a six-foot plant, which would be far cheaper than from-scratch. Prediction-if-true: six-foot hold within 4M, faster than the scratch pair. Prediction-if-false: same acq1-style destabilization (tilt collapse, declining reward, eval 0/12) -- the stilt is a knife-edge that cannot be locally deformed to a plant; rely on the scratch arms. Strongest alternative: it sheds load partially and parks on the s_i on-ramp -- watch hold_load_factor.

**gate**: Hold panel at 4M: pod_eval hold DR-0 det+sto n=6+6, >=10/12 survive with zero OC/tilt terms, six-foot stance (valid_plant or all-leg duty>=0.9), cur_p95<=1.0A.

**verdict**: Result: FAIL exactly per the pre-registered prediction-if-false -- the load-min repricing does NOT rescue the holdonly1 stilt; warm-starting it destabilizes into tilt collapse, same shape as the unrepriced acq1 continuation. Evidence: DR-0 gate hold 12/12 terminated (det 6/6 tilt_pitch, sto tilt_pitch/tilt_roll mix), duty still shows the stilt signature (2-3 legs ~1.0, others <=0.35), valid_plant 0/12, sto episodes ride the current ceiling (cur_p95 up to 2.62A, curmax 2.64A) fighting to stay up; own-DR(0.2) identical 12/12 tilt terms; reward declined monotonically (-23/-182/-334/-383); video shows the stilt stance holding then rearing/tipping. Why: with income now zero for the aloft feet, the policy tries to shift load it cannot balance -- the tripod stilt is a knife-edge basin that local gradient cannot deform into a six-foot plant (2/2 continuations from this checkpoint now destabilize: acq1 without repricing, this arm with it). What's next: the stilt-rescue lever is CLOSED -- retire holdonly1 as a parent; honest six-foot hold must come from scratch-side optimization fixes (DR-0 isolation + entropy retune arms launched this cycle) or a curriculum, not from deforming the stilt.

