# cw-amp-m2-bcinit-sec5-style05-headingsfull

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-22T21:45:01+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05-headings20

**wandb_id**: hzwzrd5c

**hypothesis**: Plain English: can the AMP walker jump straight from +/-25deg headings to FULL-CIRCLE commands (including backward walking the BC tripod init never demonstrated) in one curriculum step, or does the operator's small-set->full jump need the +/-90deg intermediate its paired dose arm (headings90) is testing? Continues from the headings20 checkpoint (--init-from-source, same sec5 minimal reward + amp 0.5/0.5, same clone-compatible obs, fresh disc per stage protocol), single lever: goal.walk_heading_max_rad 0.4363 -> -1 (full circle). Prediction-if-true: gait_valid stays >=5/6 det+sto, height_err stays in the 18-31mm band -- the walking basin generalizes around the clock. Prediction-if-false: backward/lateral demand re-collapses the gait (statue/drag/sacrificed legs) or commands are ignored (dir_err ~ uniform-heading baseline ~90deg while walking forward-only) -- then the curriculum needs the intermediate stage and/or a turn-in-place sub-skill. Strongest alternative: partial -- forward hemisphere followed, rear hemisphere ignored; per-episode dir_err spread in the report will show it.

**gate**: Discovery continuation (2M, DR-0). INFORMATIVE-PASS = gait_valid >=5/6 det+sto at own (full-circle) heading range, no new sacrificed legs, height_err in the 18-31mm band, real net travel (fwd med >=0.3m/15s), dir_err meaningfully below the ~90deg ignore-the-command baseline for uniform full-circle headings. FAIL-collapse = statue/drag/sacrificed legs or height_err climbing toward 59-85mm. PARTIAL (informative) = walks but rear-hemisphere commands ignored (bimodal per-episode dir_err).

**verdict**: The BC-init AMP walker jumped straight from +/-25deg headings to FULL-CIRCLE commands (including backward, never demonstrated by the BC init) in one curriculum step -- the intermediate +/-90deg rung is unnecessary. DR-0 gate at own full-circle range: gait_valid 6/6 det + 6/6 sto, zero sacrificed legs/terminations; det prog med 1.19 / slip 2.77 / fwd 0.47m, sto 0.87 / 3.57 / 0.56m -- all pre-registered bars met (travel >=0.3m, height_err 20.5mm inside 18-31 band all 2M, no crouch). Direction: NOT bimodal -- rear-hemisphere commands are followed (worst det episode dir_err mean 39deg, wrong-way frac <=0.11 det; along/cmd 1.0-1.4), det dir_err med ~33deg / sto ~50deg, far below the ~90deg ignore-the-command baseline. Matches or beats the headings90 intermediate rung run in parallel (det prog 1.13 / slip 3.03 / dir 35.2) -- so the operator's small-set->full jump WORKS from a walking basin. Reward rose 5.9->289 then plateaued (no continuation case); disc healthy unsaturated (d_real 0.50 / d_fake -0.80, style_reward 0.22). Frame strips watched (det_0, det_2): upright six-leg cycling, level body, real off-axis displacement, no flag leg/drag. Weak axis stated bluntly: sto direction adherence is loose (med ~50deg, p90 up to 146deg on single episodes) -- walks everywhere it is told only roughly; adherence tightening is a later stage gate. hardware-ready: no (2M discovery, DR-0). Next: stage-3 single-lever pair from THIS checkpoint -- speed range (0-0.25 m/s incl. stop/start, first stop demand) and yaw-rate commands (+/-0.5 rad/s), launched this cycle.

