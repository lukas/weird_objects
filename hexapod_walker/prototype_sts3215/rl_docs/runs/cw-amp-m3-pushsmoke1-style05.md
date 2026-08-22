# cw-amp-m3-pushsmoke1-style05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T23:23:36+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05-headingsfull

**wandb_id**: ylfbz334

**hypothesis**: Plain English: the mid-episode shove smoke that just PASSED on the task-only (noamp) walker, now on the AMP-style twin -- does the style reward, a discriminator anchored to teacher_v2's UNDISTURBED walking clips, tolerate or actively fight push-recovery transients (stumble/catch-step motions that appear in no demonstration)? This is the first axis in the M2/M3 envelope where style could plausibly HURT rather than be neutral (heading and speed axes all measured style-neutral). Clone of style05-speedrange's config with speed pinned back to fixed 0.08 (making the task cfg identical to pushsmoke1-noamp-r4's) plus the single new lever dr.ext_push_prob=1.0; init from the style05-headingsfull checkpoint (same stage as r4's noamp-headingsfull init), fresh disc, 2M discovery, judged jointly against r4's measured shape. Prediction-if-true (style tolerant): reward rises, training tilt terminations fall toward r4's 42->15 pitch / 27->7 roll shape, DR-0 own-cfg gate gait_valid >=5/6 with topple counts comparable to r4 (1 det / 3 sto), amp/style_reward stays unsaturated. Prediction-if-false (style vetoes recovery): tilt terminations stay high/flat while task reward stalls or style reward collapses around pushes, or the policy goes rigid/crouched to stay in-distribution (height_err drop, statue fingerprint). Strongest alternative: a wash -- style neither helps nor hurts, matching every other axis tested; that still clears style to ride along into M3 push hardening.

**gate**: Style-interaction smoke (2M, DR-0, dr.ext_push_prob=1.0). PASS = r4's own smoke bar (finite rising reward the whole 2M, DR-0 own-cfg gate gait_valid >=3/6 det with visible six-leg cycling and net travel, pushed episodes show some recovery not instant collapse every time) AND joint read vs r4: training tilt-termination trend falling and gate topple counts within ~2x of r4's (1/6 det, 3/6 sto) => style is tolerant, M3 hardening may keep the style channel. FAIL-styleveto = terminations not falling / gate topples >2x r4 / crouch-statue fingerprint (height_err collapsing out of the 18-31mm walking band) => style fights off-distribution recovery; M3 hardening proceeds noamp or with style annealed. FAIL-mechanism = NaN/crash.

