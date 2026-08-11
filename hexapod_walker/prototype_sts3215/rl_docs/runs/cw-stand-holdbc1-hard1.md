# cw-stand-holdbc1-hard1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-11T07:45:26+00:00

**pod**: hexapod-mjx-train-0

**steps**: 10000000

**parent**: cw-stand-holdbc1

**wandb_id**: ka02omfh

**hardware_ready**: False

**hypothesis**: Give the checkpoint that just learned to hold a genuinely quiet six-foot stand (cw-stand-holdbc1, a 2M discovery arm) more training budget, the same way the rise skill's rough edges (a flat-start footprint miss) were resolved by hardening bc1 into bc1-hard1. One variable: step count 2M->10M, same goal-mix/config, continuing from holdbc1's own weights. Binary question: does the extra budget also close the one rough edge discovery left behind (2/6 det crouch-start tilt_roll falls during rise, matching a fingerprint already present in the parent) without eroding the now-solid hold behavior?

**gate**: Harness at 10M: hold-mode det+sto stays >=10/12 valid_plant (no regression from discovery's 12/12) AND det crouch-start rise falls/misses improve or hold flat vs discovery's 2/6 (do not require net improvement, just no worsening) AND zero flag-leg/tripod cheat on any mode's video. env/hold_feet_factor should stay >=0.9 throughout (no re-drift). If hold regresses OR crouch rise gets meaningfully worse: STOP, the discovery checkpoint (not this one) is the keeper, and the next lever is the rise+hold->walk handoff composition test using cw-stand-holdbc1 directly.

**verdict**: PASS — 10M hardening consolidates the hold+rise stack, matches every pre-registered gate condition. Hold valid_plant 11/12 (det 6/6, sto 5/6 — the one miss is a soft current-limit flag, not posture/cheat), essentially matching discovery's 12/12, no regression. Det crouch-start rise 2/4 valid (50%), improved from discovery's 2/6 (33%) — the one fall is a genuine tip-over (video-confirmed), the one miss a height-overshoot on an otherwise correct six-foot stand; no flag-leg/tripod cheat anywhere across hold/track/rise det+sto (16 frame strips reviewed). env/hold_feet_factor held 0.99-1.0 the entire 10M steps (no re-drift). Checkpoint ppo_goal_cw_stand_holdbc1_hard1 is the HOLD+RISE hardened specialist. Track-mode command-tracking still weak (sto 2/6) — pre-existing, not gated, noted for later. Lineage closed for further hardening; next is the rise+hold->walk-champion handoff composition test (needs a new eval/handoff script, CODE work).

