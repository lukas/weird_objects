# cw-dep-vref1-r1-deadband

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T07:01:40+00:00

**pod**: hexapod-mjx-train-10

**steps**: 8000000

**parent**: cw-dep-vref1-r1

**wandb_id**: gnnetgrj

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: vref1-r1 (contract-exact obs + 25deg tilt) PASSed and is the leading checkpoint for tonight's hardware attempt #2, but it has never been exposed to servo deadband (1-3x, the envelope that composes for free on every other walk lineage tested tonight). Real STS3215s have dead-zone response; if vref1-r1's contract-exact velocity obs interacts badly with a sluggish/deadzone servo (timing mismatch between commanded and measured velocity), that would matter MORE for hardware than any sim metric. Per P0 rule 3, k_current=0 (hardware measured walking cheaper than standing; don't price current on dep-line arms). If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band; deadband composes free like it did on every other lineage. If-false: contract-exact velocity obs + deadband interact badly (new failure mode not seen on legacy-obs lineages) -- flag before hardware, do not deploy without this exposure.

**gate**: Own-cfg (dr.deadband_scale=1.0,3.0, contract obs, 25deg tilt) det+sto 6/6 @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band +-20%; DR0 no-deadband retention det 6/6 gv matching vref1-r1 exactly; frames watched det

