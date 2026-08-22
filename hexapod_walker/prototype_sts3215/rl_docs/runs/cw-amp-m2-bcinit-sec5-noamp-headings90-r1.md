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

**verdict**: Task-only twin ALSO survives the +/-90deg intermediate heading dose cleanly -- noamp does NOT collapse first, so style is still NOT functionally protective at this dose (matches the headings20 finding, just a modest/consistent edge, not a rescue). DR-0 gate: gait_valid 6/6 det+sto, zero sacrificed legs/terminations, height_err_end_mm med 4.5 det / 1.8 sto (no crouch). prog med det 1.12 / sto 0.85 -- essentially identical to the style05 sibling's 1.13/0.89 (within noise). dir_err med det 39.2 / sto 51.9 vs style05's 35.2/49.6 -- noamp slightly worse on direction but both well under the ~55deg gate ceiling. Slip mixed: noamp better on det (2.85 vs 3.03) but worse on sto (3.94 vs 3.69) and lower sto fwd travel (0.48m vs 0.60m) -- a wash, not a clear ranking either way. Reward rose monotonically 53.1->382.5 (higher final than style05's 269.6, but the AMP style_reward component inflates style05's own scale so raw ep_rew isn't comparable across arms). Joint read with style05-headings90: at this dose, style-vs-noamp remains a modest/inconsistent edge, not a functional rescue -- consistent with the headings20 read. hardware-ready: no (2M continuation, DR-0). Note: the parallel style05 lineage already advanced past this rung (headingsfull PASSED, jumping straight to full-circle and matching/beating headings90) -- this noamp result closes the noamp side's own record of the same rung for completeness; no further noamp-lineage continuation is queued since the track's forward motion is now on the style05-headingsfull->stage-3 line.

