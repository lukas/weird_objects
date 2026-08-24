# cw-arch-hist16-dep1-c1-joyfullcurr14-certfreeze-v8

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-24T20:12:59+00:00

**pod**: hexapod-mjx-train-5

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr13-certfreeze-v7

**wandb_id**: tvps19cg

**hypothesis**: Plain English: V7's turn/reversal diet was meant to harden the WIDE-heading buckets (side90+) against the joygate's stress_mix falls, but it also landed on the still-front-cone b1/b2 rungs, whose own stop-settle cert then never passed again -- the frontier got stuck at b1 for the entire 40M-step v7 run and the wide-heading hardening never trained at all. This run tests the scope-fixed V8 table (identical diet, starting at side90_20s instead of front45_20s) with the exact same warm start/cert-freeze/reward recipe as v7, held at v7's own 25 Hz control rate (--allow-legacy-control-hz; see OPERATOR_QUESTIONS.md 08-24 ~20:0x -- this keeps the comparison to v7 single-lever, since a concurrent cycle is separately still debugging this same lineage's 100 Hz warm-start rate conversion). Single lever: --walk-curriculum-version 7 -> 8. Prediction-if-true: frontier promotes past b1 again (like v6's b0->b5) AND the held-out joygate improves toward v6's own bar (falls <=2/48) without the front45 stop-creep regression v7 showed. Prediction-if-false: if frontier still stalls despite the scope fix, the stop-gate itself (0.015 m/s in a 20s window) is too tight for ANY reversal-bearing bucket regardless of which rung it starts at, pointing next at a settle-time/grace-window fix rather than bucket placement.

**gate**: PASS: frontier promotes past b1 to at least b3 (side90 opens) AND held-out 60s joygate falls <=2/48 AND DR-0 det gait_valid stays 6/6 with no leg sacrifice. PARTIAL: frontier promotes past b1 but joygate/DR-0-gait doesn't fully clear (genuine trade, or improvement without clearing). FAIL: frontier still stalls at b1 (the scope fix didn't touch the mechanism) -- points next lever at the stop-settle grace window itself, not bucket placement.

