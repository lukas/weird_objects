# cw-amp-m2-bcinit-sec5-noamp-seed1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-22T22:21:55+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-noamp

**wandb_id**: kb7pwf37

**hypothesis**: Plain English: task-only twin of cw-amp-m2-bcinit-sec5-style05-seed1 -- does the BC-init-alone escape from the crouch-statue basin (no AMP style at all) also reproduce on an independent seed, matching the seed-7 result where noamp PASSED (gait_valid 6/6 det+sto, real fwd travel) just slightly worse than the style05 twin on every axis? Single lever vs the seed-7 parent: --seed 7->1, everything else byte-identical (BC-clone init, sec5 minimal reward, clone-compatible obs, zero AMP flags, 2M discovery, DR-0). Paired with the style05-seed1 twin: gives the FIRST 2-seed style-vs-noamp comparison on a genuinely locomoting actor, testing whether style's modest seed-7 edge is a recipe property or n=6 noise.

**gate**: Discovery (2M). PASS/reproduces = DR-0 gate det+sto gait_valid stays >=5/6 (parent 6/6 both), no new sacrificed legs, height_err stays in the 18-31mm band, det fwd/15s within ~30% of parent's 0.64m. FAIL = statue/crouch basin reappears on this seed.

**verdict**: Seed-robustness check for the noamp (task-only) BC-init walking result reproduces cleanly. DR-0 gate: gait_valid 6/6 det+sto (matches seed7 parent's 6/6 both), zero sacrificed legs, det prog med 0.89/slip 2.48/fwd 0.53m (parent seed7: 1.09/2.13/0.64m -- softer but same walking-basin regime, no crouch), sto prog med 0.44/slip 6.24/fwd 0.25m (parent 0.52/5.38/0.24m -- comparable, fwd matches almost exactly). Frame-strip clean six-leg alternating-tripod cycling, no drag/skate/flag-leg. Confirms noamp also reproduces on a second seed -- both style05 and noamp lines are now seed-checked (2/2 each), the BC-init escape is a robust recipe property. hardware-ready: no (2M discovery, DR-0, forward-only, seed-check only).

