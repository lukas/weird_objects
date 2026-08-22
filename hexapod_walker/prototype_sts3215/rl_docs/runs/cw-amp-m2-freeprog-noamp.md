# cw-amp-m2-freeprog-noamp

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T13:40:23+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-pilot-noamp

**wandb_id**: ova8jg2e

**hypothesis**: Can a from-scratch policy discover real stepping once standing still stops paying? The 40M M2 pilot pair proved the legacy walk reward's optimum is a statue (one leg triad planted, one airborne; ALL reward growth was stand-income). This control re-runs the no-AMP pilot config from scratch with the bank-calibrated freeprog anti-slip pricing (statue nets -238/ep vs honest gait +558; test_slipwalk_stork_statue_is_priced_out PASS) plus the pre-registered branch-(iii) envelope narrowing (speed 0-0.25 m/s, yaw +/-0.5, pure-walk diet). Prediction-if-true: by 2M the det video shows six legs cycling and real travel (median fwd >= 0.10 m/15 s), unlike the statue fingerprint. Prediction-if-false: freeze/stall fingerprint repeats (cw-nobc-slipwalk1-r1 froze at 2M under this pricing at a fixed command) — which makes the style05 twin the decisive arm: pricing alone insufficient, motion-prior gradient required. Strongest alternative: the idle/park charges destabilize early training into falls instead of stepping.

**gate**: Discovery (2M, DR-0 harness walk mode, 6 det + 6 sto episodes, own cfg): PASS = zero terminations majority of episodes AND median fwd travel >= 0.10 m/15 s AND gait_valid >= 4/6 det with video showing all six legs cycling (statue/flag/stilt = FAIL regardless of scalars). Judged as the matched control for cw-amp-m2-freeprog-style05; no SKILLS/champion updates. Statue fingerprint here + stepping in the style05 twin = the first real style-vs-control win and the Wave-1 unlock evidence.

