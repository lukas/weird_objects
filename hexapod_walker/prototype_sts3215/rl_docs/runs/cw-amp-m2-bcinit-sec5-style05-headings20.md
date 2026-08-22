# cw-amp-m2-bcinit-sec5-style05-headings20

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-22T21:23:03+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05

**wandb_id**: ubydzfij

**hypothesis**: Plain English: does the just-proven BC-init + sec5-reward walking (bcinit-sec5-style05 PASS: gait_valid 6/6, real fwd travel, no crouch) survive being asked to walk in DIFFERENT DIRECTIONS, or does the exploration noise needed to learn turning re-collapse it into the statue basin like full +/-180deg headings did to the joystick track's phase-RL lineage (cw-dep-bcgait4-phasedir1 FAIL) from step 1? Continues from the style05 checkpoint (--init-from-source, same weights, same sec5 minimal reward + amp task/style 0.5/0.5, same clone-compatible obs/env), single lever change: goal.walk_heading_max_rad 0.0 -> 0.4363 rad (25 deg, a SMALL heading set per the operator's own untried staged-curriculum design fb_20260822T003132 -- stage 1 of forward-only -> small heading set -> full headings -> irregular, applied here for the first time on the AMP track instead of the phase-RL joystick lineage where it was pre-registered but superseded by that lineage reaching its own DONE gate). 2M discovery continuation, DR-0, seed unchanged (7).

**gate**: Discovery continuation (2M, judged on det video + DR-0 gate harness read at the new nonzero heading range). INFORMATIVE-PASS = gait_valid stays >=5/6 det+sto, no new sacrificed legs, height_err stays near the 18-31mm walking-basin band (not climbing toward the 59-85mm crouch signature), fwd/prog on off-axis headings comparable to the forward baseline (allow up to ~30% softening for the added turning demand). FAIL-collapse = gait degrades toward statue/drag when forced to turn -- heading diversity re-triggers the from-scratch exploration failure even from a walking init; motivates a SMALLER first stage (e.g. 10deg) or an explicit turn-in-place sub-skill before blending into the main command distribution.

**verdict**: Stage 1 of the heading curriculum works: the BC-initialized AMP walker learned to walk toward +/-25deg commanded headings without losing its gait -- and the added diversity made it MORE robust, not less. DR-0 gate at its own heading range: gait_valid 6/6 det + 6/6 sto, zero sacrificed legs, zero terminations; det prog med 1.30 / slip 2.17 / fwd 0.64m vs forward-only parent style05's 1.16 / 1.88 / 0.69m (prog up, slip mildly up, well inside the registered 30% allowance); sto improved OUTRIGHT (prog 0.58->0.93, slip 4.71->3.38, fwd 0.23->0.65m) and dir_err improved BOTH modes (det 33.8->26.9deg, sto 61.9->48.2deg). env/height_err_mm held 22-26mm the whole 2M (never the 59-85mm crouch signature), reward rose monotonically 19->247, disc healthy (d_real 0.47 / d_fake -0.75 separated, style_reward 0.25, gp finite). Frame strips watched (walk_det_0 near-straight, walk_det_4 off-axis): upright six-leg cycling with real checkerboard displacement, no statue, no drag, no flag leg. Blunt weak axis: direction ADHERENCE is still loose (det 26.9deg on a 25deg command range) -- it walks well but only roughly where told; later stages must convert that into command-following and we watch dir_err explicitly. Prediction-if-true fired exactly; heading diversity did NOT re-trigger the exploration collapse. Hardware-ready: no (M2 sim milestone work). Next: stage-2 dose pair launched from this checkpoint -- headings90 (+/-90deg, adds lateral) and headingsfull (full circle, adds backward), single lever each, 2M discovery, deciding whether the operator's small-set->full curriculum jump survives in one step or needs the intermediate.

