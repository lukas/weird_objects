# cw-amp-m2-bcinit-sec5-style05-headings90

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-22T21:40:50+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05-headings20

**wandb_id**: 7dx5sfex

**hypothesis**: Plain English: now that the AMP walker survived +/-25deg headings (headings20 PASS, gait_valid 12/12, sto and dir_err IMPROVED vs the forward-only parent), does it survive the intermediate curriculum dose of +/-90deg -- which for the first time demands genuinely LATERAL walking the BC tripod init never demonstrated? Continues from the headings20 checkpoint (--init-from-source, same sec5 minimal reward + amp 0.5/0.5, same clone-compatible obs, fresh disc per stage protocol), single lever: goal.walk_heading_max_rad 0.4363 -> 1.5708. Prediction-if-true: gait_valid stays >=5/6 det+sto, height_err stays in the 18-31mm band, dir_err does not blow past ~55deg. Prediction-if-false: lateral demand re-collapses the gait (statue/drag/sacrificed legs) or the policy ignores off-axis commands entirely (dir_err ~ command range while walking straight). Strongest alternative: it walks but adherence stalls -- style/task weights or missing yaw-cmd obs become the next lever. Paired dose arm: headingsfull tests the operator's small-set->full jump in one step.

**gate**: Discovery continuation (2M, DR-0). INFORMATIVE-PASS = gait_valid >=5/6 det+sto at own heading range, no new sacrificed legs, height_err in the 18-31mm band, prog med within ~30% of headings20's (det 1.30/sto 0.93), dir_err not worse than headings20's (det 26.9/sto 48.2) by more than ~15deg. FAIL-collapse = statue/drag/sacrificed legs or height_err climbing toward 59-85mm.

**verdict**: Style twin SURVIVES the +/-90deg intermediate heading dose (first lateral-walking demand) cleanly, matching its noamp-headings90-r1 sibling in flight. DR-0 gate: gait_valid 6/6 det+sto, zero sacrificed legs/terminations, height_err_end_mm med 7.4 det / 4.7 sto (training env/height_err_mm held 14-26mm all 2M, no crouch). prog med det 1.13 / sto 0.89 -- within the pre-registered 30% band of headings20's 1.30/0.93. dir_err med det 35.2 / sto 49.6 -- within 15deg of headings20's 26.9/48.2 (both bands honored). Slip rose (det 3.03 vs headings20's 2.17, sto 3.69 vs 3.38) -- the honest cost of genuine lateral demand, not a pathology; no drag/skate signature on the frame strips (all six legs cycling, visible turning displacement across the sheet). Reward rose monotonically 51.8->269.6. Video watched det_2 + contact sheet: clean six-leg gait, real off-axis translation. hardware-ready: no (2M continuation, DR-0). Next: read jointly with noamp-headings90-r1 once it lands for the style-vs-noamp delta at this harder dose; if both clear, stage headingsfull is the natural next rung (already running).

