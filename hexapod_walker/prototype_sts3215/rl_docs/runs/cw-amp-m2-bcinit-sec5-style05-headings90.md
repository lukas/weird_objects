# cw-amp-m2-bcinit-sec5-style05-headings90

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T21:40:50+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05-headings20

**wandb_id**: 7dx5sfex

**hypothesis**: Plain English: now that the AMP walker survived +/-25deg headings (headings20 PASS, gait_valid 12/12, sto and dir_err IMPROVED vs the forward-only parent), does it survive the intermediate curriculum dose of +/-90deg -- which for the first time demands genuinely LATERAL walking the BC tripod init never demonstrated? Continues from the headings20 checkpoint (--init-from-source, same sec5 minimal reward + amp 0.5/0.5, same clone-compatible obs, fresh disc per stage protocol), single lever: goal.walk_heading_max_rad 0.4363 -> 1.5708. Prediction-if-true: gait_valid stays >=5/6 det+sto, height_err stays in the 18-31mm band, dir_err does not blow past ~55deg. Prediction-if-false: lateral demand re-collapses the gait (statue/drag/sacrificed legs) or the policy ignores off-axis commands entirely (dir_err ~ command range while walking straight). Strongest alternative: it walks but adherence stalls -- style/task weights or missing yaw-cmd obs become the next lever. Paired dose arm: headingsfull tests the operator's small-set->full jump in one step.

**gate**: Discovery continuation (2M, DR-0). INFORMATIVE-PASS = gait_valid >=5/6 det+sto at own heading range, no new sacrificed legs, height_err in the 18-31mm band, prog med within ~30% of headings20's (det 1.30/sto 0.93), dir_err not worse than headings20's (det 26.9/sto 48.2) by more than ~15deg. FAIL-collapse = statue/drag/sacrificed legs or height_err climbing toward 59-85mm.

