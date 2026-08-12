# cw-stand-footlow2-hard1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-12T13:35:53+00:00

**pod**: hexapod-mjx-train-0

**steps**: 10000000

**parent**: cw-stand-footlow2-hard1

**wandb_id**: 1ppsq8hv

**hardware_ready**: False

**hypothesis**: Make the deployed stance a reproducible recipe, not a lucky seed: cw-stand-footlow2-hard1 just passed all four pre-registered gate clauses at once (clean cold-start rise incl. flat, zero real hold park, 12/12 lower, and eval_session hard gates including a full 148mm rise where the deployed holdbc1_hard1 stalls at 55mm) -- this arm is an independent-seed twin of the IDENTICAL 10M consolidation recipe (same footlow2-r1 warm-start, same anchor stack, only --seed changes) to check whether that result reproduces. Prediction-if-true: the same four clauses pass again with comparable numbers (cold rise <=5mm all start kinds incl. flat, all-six-foot hold duty>=0.5 with no real park, lower >=10/12, eval_session hard gates PASS) -- promotion-over-holdbc1_hard1 becomes a well-supported call, not a one-seed fluke. Prediction-if-false: one clause misses on this seed, most likely a return of the historical rise/hold seesaw (hold park reopens past the 2mm cosmetic bar, or a cold-start rise stall reappears) -- meaning the anchor stack is still seed-sensitive and the PASS needs a second independent recipe check before any promotion, not just more steps.

**gate**: PASS if, on this seed: all cold-start (flat/bridge/crouch, non-rsi) det rises are valid_plant with h_err<=5mm (use a targeted flat-only probe if the standard draw samples no flat episodes, as hard1's did); det hold has NO real park (no foot with duty<0.5 AND end_clear>2mm); det+sto lower >=10/12 success/valid_plant; AND rl_move.sim.eval_session hard gates (falls/rise/sit) pass. FAIL if any clause misses. Quote roll_tail/drag/slip vs cw-stand-footlow2-hard1 (seed 0) in the verdict -- this is a reproduction check, not a new mechanism.

**verdict**: PASS — independent-seed twin REPRODUCES cw-stand-footlow2-hard1's PASS cleanly, all four gate clauses hold on seed 1: (1) cold det rises (bridge 2/2 h_err 2.6/1.4mm, crouch 1/1 h_err 2.5mm from the draw; flat 12/12 via a targeted probe, h_err 0.3-3.9mm, roll_tail 0.2-0.3deg) all valid_plant <=5mm; (2) det hold ZERO real park, all six feet duty 0.97-0.99 with end_clear -0.2..0.3mm (tighter than the parent's own 0.95-0.99/0.1-0.2mm); (3) lower 12/12 det+sto success, height_err 0-1.9mm, no outrigger; (4) eval_session HARD gates (no_falls/rise/sit_descends) all PASS, rise reaches 148.2mm under the interactive ramp -- matching the parent's 148mm almost exactly. Visual-quality vs cw-stand-footlow2-hard1 (seed 0): det hold drag 138mm/roll_tail 0.0deg (parent 136mm/0.1deg), det rise drag 442mm/roll_tail 4.9deg (parent 434mm/5.1deg, both driven by one non-cold rsi episode), det lower drag 259mm (parent 244mm), sto hold drag 1273mm/roll_tail 0.8deg (parent 1283mm/0.8deg) -- all within noise, no degradation on any axis. Video-confirmed clean six-foot stance on hold/rise/lower det strips and the flat-probe strip; zero flag-leg/park/stilt. This is the second independent seed clearing all four clauses -- the footlow2-hard1 recipe is now seed-robust (RL_PLAN's multi-seed promotion bar), not a lucky single seed. Promotion-over-holdbc1_hard1 stays a bench-readiness/operator call; this run only strengthens the sim-side case for it.

