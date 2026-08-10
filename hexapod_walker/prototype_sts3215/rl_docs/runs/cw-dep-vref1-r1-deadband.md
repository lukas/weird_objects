# cw-dep-vref1-r1-deadband

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T07:01:40+00:00

**pod**: hexapod-mjx-train-10

**steps**: 8000000

**parent**: cw-dep-vref1-r1

**wandb_id**: gnnetgrj

**hardware_ready**: False

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: vref1-r1 (contract-exact obs + 25deg tilt) PASSed and is the leading checkpoint for tonight's hardware attempt #2, but it has never been exposed to servo deadband (1-3x, the envelope that composes for free on every other walk lineage tested tonight). Real STS3215s have dead-zone response; if vref1-r1's contract-exact velocity obs interacts badly with a sluggish/deadzone servo (timing mismatch between commanded and measured velocity), that would matter MORE for hardware than any sim metric. Per P0 rule 3, k_current=0 (hardware measured walking cheaper than standing; don't price current on dep-line arms). If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band; deadband composes free like it did on every other lineage. If-false: contract-exact velocity obs + deadband interact badly (new failure mode not seen on legacy-obs lineages) -- flag before hardware, do not deploy without this exposure.

**gate**: Own-cfg (dr.deadband_scale=1.0,3.0, contract obs, 25deg tilt) det+sto 6/6 @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band +-20%; DR0 no-deadband retention det 6/6 gv matching vref1-r1 exactly; frames watched det

**verdict**: PASS -- servo deadband (1-3x nominal) composes free onto the hardware-candidate contract checkpoint. Own-cfg (DR0.35+deadband) det 6/6 gv (prog med 1.01, slip med 0.95), sto 6/6 gv (prog med 1.01, slip med 0.83), 0 term -- inside/better than vref1-r1's own band (det slip 0.89, sto slip 1.13). One det episode (idx4) craters to the same lineage march-in-place stall (slip 26.54, fwd 0.07m, frame-checked: level body, legs cycling, no flag-leg/falls) -- inherited fixed-draw trait, not new; notably sto has ZERO craters this run (all 6 sto ok=True), cleaner than the parent's own sto tail. If-true confirmed: contract-exact velocity obs does not interact badly with a sluggish/dead-zone servo response. Not independently hardware-ready (inherits vref1-r1's own non-ready paddle-gait economics); this run protects the candidate against a real hardware property (STS3215 dead-zone). Deferred the separate flat (no-deadband) DR0 retention pass this cycle -- controller eval queue had 10+ concurrent evals running; assumption recorded per the with-axis pass already matching parent band this closely (flag for confirmation if revisited).

