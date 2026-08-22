# cw-amp-m2-bcinit-sec5-style05-headingsfull

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-22T21:45:01+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05-headings20

**hypothesis**: Plain English: can the AMP walker jump straight from +/-25deg headings to FULL-CIRCLE commands (including backward walking the BC tripod init never demonstrated) in one curriculum step, or does the operator's small-set->full jump need the +/-90deg intermediate its paired dose arm (headings90) is testing? Continues from the headings20 checkpoint (--init-from-source, same sec5 minimal reward + amp 0.5/0.5, same clone-compatible obs, fresh disc per stage protocol), single lever: goal.walk_heading_max_rad 0.4363 -> -1 (full circle). Prediction-if-true: gait_valid stays >=5/6 det+sto, height_err stays in the 18-31mm band -- the walking basin generalizes around the clock. Prediction-if-false: backward/lateral demand re-collapses the gait (statue/drag/sacrificed legs) or commands are ignored (dir_err ~ uniform-heading baseline ~90deg while walking forward-only) -- then the curriculum needs the intermediate stage and/or a turn-in-place sub-skill. Strongest alternative: partial -- forward hemisphere followed, rear hemisphere ignored; per-episode dir_err spread in the report will show it.

**gate**: Discovery continuation (2M, DR-0). INFORMATIVE-PASS = gait_valid >=5/6 det+sto at own (full-circle) heading range, no new sacrificed legs, height_err in the 18-31mm band, real net travel (fwd med >=0.3m/15s), dir_err meaningfully below the ~90deg ignore-the-command baseline for uniform full-circle headings. FAIL-collapse = statue/drag/sacrificed legs or height_err climbing toward 59-85mm. PARTIAL (informative) = walks but rear-hemisphere commands ignored (bimodal per-episode dir_err).

