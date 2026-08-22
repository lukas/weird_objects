# cw-amp-m2-bcinit-sec5-style05-seed1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-22T22:18:46+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05

**wandb_id**: p7t39bjo

**hypothesis**: Plain English: does the first non-statue from-scratch-M2 result (BC-clone init + sec5 minimal reward + AMP style, PASS on seed 7: gait_valid 6/6 det+sto, real fwd travel, height_err 18-31mm stable, no crouch) reproduce on an independent seed, or was it a lucky single-seed escape from the crouch-statue basin? Single lever: --seed 7->1, everything else byte-identical (same BC-clone init checkpoint, same sec5 minimal reward, same clone-compatible obs, same 0.5/0.5 task/style blend, 2M discovery, DR-0). This is the recipe-vs-luck question the STATUS/SKILLS entries explicitly flagged as open ('not yet proven outside n=6 noise' on the style-vs-noamp delta, and no seed-twin exists yet for the walking result itself).

**gate**: Discovery (2M). PASS/reproduces = DR-0 gate det+sto gait_valid stays >=5/6 (parent 6/6 both), no new sacrificed legs, height_err stays in the 18-31mm band (not the 59-85mm crouch signature), det fwd/15s within ~30% of parent's 0.69m. FAIL = statue/crouch basin reappears on this seed (gait_valid collapses, height_err climbs toward 59-85mm) -- the sec5+BC-init escape would then read as seed-fragile, not a robust recipe fix.

**verdict**: Seed-robustness check for the BC-init + sec5 + style walking result reproduces cleanly on a second seed. DR-0 gate: gait_valid 6/6 det+sto (matches seed7 parent's 6/6 both), zero sacrificed legs, det prog med 0.95/slip 2.44/fwd 0.57m (parent seed7: 1.16/1.88/0.69m -- softer but same walking-basin regime, no crouch), sto prog med 0.48/slip 5.49/fwd 0.24m (parent 0.58/4.71/0.23m -- comparable). Frame-strip (walk_det_5) clean six-leg alternating-tripod cycling, visible forward displacement, no drag/skate/flag-leg. Confirms the BC-init escape from the from-scratch statue basin is a RECIPE property, not a seed-7 fluke -- second independent seed lands in the same walking basin with the same reward. hardware-ready: no (2M discovery, DR-0, forward-only, seed-check only).

