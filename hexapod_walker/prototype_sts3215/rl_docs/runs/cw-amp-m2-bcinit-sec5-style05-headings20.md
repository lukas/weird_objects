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

**verdict**: Heading-diversity stage 1 (0 -> 25deg / 0.4363rad) SURVIVES cleanly, confirming the BC-init walking basin is robust to turning demand, not fragile like the joystick track's full-180-from-rung-1 phase-RL failure. DR-0 gate: gait_valid still 6/6 det+sto, zero sacrificed legs, det prog med 1.30 (UP from the forward-only baseline's 1.16), slip med 2.17 (up from 1.88, a real but modest +15% cost of the added turning demand, well inside the pre-registered 30% allowance); sto prog med 0.93 (up from 0.58), slip 3.38 (down from 4.71 -- sto actually IMPROVED, likely noise now helps sample the heading distribution rather than pure forward overshoot). env/height_err_mm stayed 15-26mm the whole run (walking-basin band, nowhere near the 59-85mm crouch signature). Video (walk_det_2/4 frame strips) shows continued clean six-leg alternating-tripod cycling; checkerboard track shows off-axis drift consistent with non-forward headings, no drag/skate/flag-leg. hardware-ready: no (2M continuation, DR-0, single 25deg heading stage). Next: stage 2 (wider heading range) launched this cycle to find the curriculum's actual limit.

