# cw-amp-m2-bcinit-sec5-noamp-headings90-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-22T21:49:34+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-noamp-headings20

**wandb_id**: 1lsrht17

**hypothesis**: Task-only twin of the concurrently-launched cw-amp-m2-bcinit-sec5-style05-headings90: does the noamp BC-init walker also survive the intermediate +/-90deg heading dose (the first stage demanding genuinely lateral walking, which the BC tripod init never demonstrated), or does style become protective exactly where the exploration/turning demand gets hard? Continues from the noamp-headings20 checkpoint (--init-from-source), single lever: goal.walk_heading_max_rad 0.4363 -> 1.5708, identical to the style05 sibling arm for a clean paired read. (-r1 suffix: first attempt's run name collided with a tag pushed by a stale earlier retry of this same launch.)

**gate**: Discovery continuation (2M, judged on det video + DR-0 gate harness, read JOINTLY with the style05-headings90 sibling). INFORMATIVE-PASS = gait_valid stays >=5/6 det+sto, height_err stays in the 14-26mm walking-basin band, dir_err does not blow past ~55deg. FAIL-collapse = gait degrades (statue/drag/sacrificed legs) or the policy ignores off-axis commands. Joint read: style-vs-noamp delta at this harder dose is the batch's primary measurement -- if noamp collapses first, style is functionally protective for the first time.

**verdict**: noamp twin ALSO survives the +/-90deg intermediate heading dose cleanly, matching its style05-headings90 sibling (already verdicted PASS by a concurrent cycle). DR-0 gate: gait_valid 6/6 det+sto, zero sacrificed legs/terminations, det prog med 1.12 (vs headings20's 1.09, essentially flat), slip med 2.85 (vs headings20's 2.45, a real but modest cost); sto prog med 0.85 (vs 0.79), slip 3.94 (vs 4.36, slightly better). Frame-strip (walk_det_1) shows clean six-leg alternating-tripod cycling, visible off-axis translation, no drag/skate/flag-leg. JOINT READ vs the style05-headings90 sibling (det prog 1.13/slip 3.03, sto prog 0.89/slip 3.69): the two arms are now essentially TIED -- noamp actually has slightly LOWER det slip (2.85 vs 3.03) though slightly higher sto slip (3.94 vs 3.69), prog matches within noise both modes. This CLOSES the 'is style protective under harder turning demand' question in the negative for this dose: the modest style edge measured at forward-only and 25deg has washed out by 90deg -- style is not (yet) earning a functional advantage, only the earlier cosmetic-adjacent margin. hardware-ready: no (2M continuation, DR-0). Batch conclusion: BOTH style05 and noamp survive 0->25->90deg cleanly with no crouch/collapse at any stage -- the BC-init basin is robust to heading diversity up to at least 90deg regardless of style. Next: headingsfull (concurrent cycle, already running for style05) is the natural next rung; a noamp-headingsfull twin would complete the paired read if GPU budget allows.

