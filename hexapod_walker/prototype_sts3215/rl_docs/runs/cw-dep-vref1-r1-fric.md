# cw-dep-vref1-r1-fric

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T07:00:22+00:00

**pod**: hexapod-mjx-train-11

**steps**: 8000000

**parent**: cw-dep-vref1-r1

**wandb_id**: 55ajzn44

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: vref1-r1 is the leading checkpoint for tonight's hardware attempt #2 but has never been exposed to friction variation (0.4-1.6x, the envelope proven free on other walk lineages), and the real floor's friction is unknown/unmeasured. Per P0 rule 3, k_current=0 (don't price current on dep-line arms; hardware walking measured cheaper than standing). If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- friction composes free like it did elsewhere, one less unknown before deployment. If-false: contract-exact velocity obs is more friction-sensitive than legacy obs (plausible: honest measured velocity could amplify a friction-induced slip/measurement mismatch) -- flag before hardware.

**gate**: Own-cfg (dr.friction_scale=0.4,1.6, contract obs, 25deg tilt) det+sto 6/6 @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band +-20%; DR0 no-friction-var retention det 6/6 gv matching vref1-r1 exactly; frames watched det

