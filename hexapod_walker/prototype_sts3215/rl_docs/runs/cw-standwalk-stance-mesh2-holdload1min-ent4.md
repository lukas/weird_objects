# cw-standwalk-stance-mesh2-holdload1min-ent4

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-25T08:43:15+00:00

**pod**: hexapod-mjx-train-0

**steps**: 6000000

**parent**: cw-standwalk-stance-mesh2-holdload1min-s1

**hypothesis**: Plain English: does keeping exploration pressure high stop the policy from collapsing onto its belly and freezing? Single lever off the holdload1min scratch recipe: ent-coef 0.005 -> 0.02 (4x; the 0.005 value is an untouched primitive-era default never retuned for mesh/100Hz -- rung-2 candidate (c)). Rationale: s1's belly-freeze is a low-variance do-nothing attractor that training reward walked DOWN into (-94 -> -627) while the aligned optimum (+1471 six-foot hold) sits at the episode's own start pose -- a classic premature-variance-collapse story. Sustained entropy keeps the policy sampling near-plant actions long enough for the load-min income to be discovered and harvested. Prediction-if-true: income turns positive and a six-foot hold emerges by 6M. Prediction-if-false: still freezes (or now falls more from added noise under DR 0.2) -- entropy dose is not the lever; read jointly with the dr0 sibling to pick between DR-ramp machinery and a hold curriculum for rung-5.

**gate**: Same hold panel as holdload1min at 6M: pod_eval hold DR-0 det+sto n=6+6, >=10/12 survive with zero OC/tilt terms, six-foot stance (valid_plant or all-leg duty>=0.9), cur_p95<=1.0A; own-DR(0.2) read alongside.

**refused_reason**: discovery runs cap at 2000000 steps (asked 6000000): the question is 'did qualitatively correct behavior emerge?' - continue as --phase hardening with --evidence.

