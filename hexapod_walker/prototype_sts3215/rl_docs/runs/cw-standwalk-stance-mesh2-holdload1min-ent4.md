# cw-standwalk-stance-mesh2-holdload1min-ent4

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-25T08:44:08+00:00

**pod**: hexapod-mjx-train-0

**steps**: 6000000

**parent**: cw-standwalk-stance-mesh2-holdload1min-s1

**wandb_id**: 39kxul3i

**hypothesis**: Plain English: does keeping exploration pressure high stop the policy from collapsing onto its belly and freezing? Single lever off the holdload1min scratch recipe: ent-coef 0.005 -> 0.02 (4x; the 0.005 value is an untouched primitive-era default never retuned for mesh/100Hz -- rung-2 candidate (c)). Rationale: s1's belly-freeze is a low-variance do-nothing attractor that training reward walked DOWN into (-94 -> -627) while the aligned optimum (+1471 six-foot hold) sits at the episode's own start pose -- a classic premature-variance-collapse story. Sustained entropy keeps the policy sampling near-plant actions long enough for the load-min income to be discovered and harvested. Prediction-if-true: income turns positive and a six-foot hold emerges by 6M. Prediction-if-false: still freezes (or now falls more from added noise under DR 0.2) -- entropy dose is not the lever; read jointly with the dr0 sibling to pick between DR-ramp machinery and a hold curriculum for rung-5.

**gate**: Same hold panel as holdload1min at 6M: pod_eval hold DR-0 det+sto n=6+6, >=10/12 survive with zero OC/tilt terms, six-foot stance (valid_plant or all-leg duty>=0.9), cur_p95<=1.0A; own-DR(0.2) read alongside.

**verdict**: 4x entropy does NOT rescue the load-min hold recipe: the robot still drops from its planted start onto its belly and freezes there, exactly like the s1 sibling. Evidence: 0/24 valid hold (DR-0 gate + own-DR 0.2, det+sto), zero terminations, height_err_end ~70mm every episode, hold_load_factor pinned at the 0.1 min-gate floor, cur_p95 0.5-0.6A (frozen, not fighting); det frame strip shows plant -> belly collapse by t~2s -> frozen splay. The lever mechanically fired -- train/std stayed at 1.68 all run (vs ~0.4 typical collapse) -- yet training reward still declined -99 -> -708: sustained action noise alone never re-plants six feet simultaneously, so min-over-feet income stays at the floor with no gradient home. Not an 08-21 continue case (reward falling, not rising). Joint read with dr0 sibling (FAIL, tilt-topple basin, concurrent cycle 08:57): BOTH single-lever optimization arms are dead -> per pre-registration rung-5 is hold-curriculum / pose-anchor mechanism work (bank first), while the concurrent holdprod f01/f03 arms adjudicate the reward-gradient-shape axis. Load-min income gate is now 5/5 arms dead across 4 distinct basins (rear-up OC, belly-freeze x2, tilt-topple, warm-stilt collapse) -- the min-over-feet variant is closed as a scratch-trainable income shape.

