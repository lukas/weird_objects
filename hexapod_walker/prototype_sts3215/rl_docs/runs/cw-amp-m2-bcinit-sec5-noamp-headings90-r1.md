# cw-amp-m2-bcinit-sec5-noamp-headings90-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T21:49:34+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-noamp-headings20

**wandb_id**: 1lsrht17

**hypothesis**: Task-only twin of the concurrently-launched cw-amp-m2-bcinit-sec5-style05-headings90: does the noamp BC-init walker also survive the intermediate +/-90deg heading dose (the first stage demanding genuinely lateral walking, which the BC tripod init never demonstrated), or does style become protective exactly where the exploration/turning demand gets hard? Continues from the noamp-headings20 checkpoint (--init-from-source), single lever: goal.walk_heading_max_rad 0.4363 -> 1.5708, identical to the style05 sibling arm for a clean paired read. (-r1 suffix: first attempt's run name collided with a tag pushed by a stale earlier retry of this same launch.)

**gate**: Discovery continuation (2M, judged on det video + DR-0 gate harness, read JOINTLY with the style05-headings90 sibling). INFORMATIVE-PASS = gait_valid stays >=5/6 det+sto, height_err stays in the 14-26mm walking-basin band, dir_err does not blow past ~55deg. FAIL-collapse = gait degrades (statue/drag/sacrificed legs) or the policy ignores off-axis commands. Joint read: style-vs-noamp delta at this harder dose is the batch's primary measurement -- if noamp collapses first, style is functionally protective for the first time.

