# cw-dep-vref1-r1-fric

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T07:00:22+00:00

**pod**: hexapod-mjx-train-11

**steps**: 8000000

**parent**: cw-dep-vref1-r1

**wandb_id**: 55ajzn44

**hardware_ready**: False

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: vref1-r1 is the leading checkpoint for tonight's hardware attempt #2 but has never been exposed to friction variation (0.4-1.6x, the envelope proven free on other walk lineages), and the real floor's friction is unknown/unmeasured. Per P0 rule 3, k_current=0 (don't price current on dep-line arms; hardware walking measured cheaper than standing). If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- friction composes free like it did elsewhere, one less unknown before deployment. If-false: contract-exact velocity obs is more friction-sensitive than legacy obs (plausible: honest measured velocity could amplify a friction-induced slip/measurement mismatch) -- flag before hardware.

**gate**: Own-cfg (dr.friction_scale=0.4,1.6, contract obs, 25deg tilt) det+sto 6/6 @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band +-20%; DR0 no-friction-var retention det 6/6 gv matching vref1-r1 exactly; frames watched det

**verdict**: PASS-with-caveat -- floor friction 0.4-1.6x composes onto the contract-exact hardware candidate (k_current=0 per P0). Own-cfg det+sto 6/6 gait_valid, 0 term; det slip/m med 1.23 sits ~9% over vref1-r1's own nominal upper band (1.13) but inside this run's pre-registered +-20% tolerance, sto med 1.04 within band. Named-baseline retention pass (identical eval minus the friction cfg-set) gives clean det 6/6 med 1.00 fwd 0.78m (vs vref1-r1's own 0.89, within noise) and reproduces vref1-r1's OWN single-seed (idx4) sto-only paddling stall almost exactly (prog 0.25 slip 6.58 vs vref1-r1's 0.26/5.97) -- the checkpoint itself is unchanged, friction is what pushed the elevated median. Under friction variation that same seed-4 draw's stall migrates into det too (prog~0 slip~27, fwd 0.02m) but stays a march-in-place (gait_valid True, no flag leg, no fall -- confirmed on video det_0..5), matching the already-root-caused lineage paddling-attractor class (c79 dig-in). Not hardware-ready on its own; clears friction variation as survivable (not fully cost-free) for the hardware candidate -- flagging the elevated det median so it isn't silently averaged away.

