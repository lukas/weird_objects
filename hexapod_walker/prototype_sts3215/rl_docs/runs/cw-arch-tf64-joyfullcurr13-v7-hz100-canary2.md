# cw-arch-tf64-joyfullcurr13-v7-hz100-canary2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-24T20:49:27+00:00

**pod**: hexapod-mjx-train-6

**steps**: 2000000

**parent**: cw-arch-tf64-small-joyfullcurr13-v7-hz100-canary-r1

**wandb_id**: rel6d200

**hypothesis**: Plain English: was the small-transformer's 100Hz collapse (reward -20->-609 declining, frontier stuck at b0, leg-0 locked 47/48 joygate episodes) an under-capacity problem or an attention-architecture-specific pathology? The canary-r1 read already isolates the failure away from the 100Hz rate and reward stack (a sibling MLP on the identical stack learns healthy, reward rising to 746). This arm escalates ONLY tf width/layers (1L/d64/4h/ff128 -> 2L/d128/8h/ff256, the exact config that DID learn to walk at 40M pre-100Hz as cw-arch-tf-r1-hard1), same V7 certfreeze recipe, same hist64/control.hz=100 window, same 2M canary budget, per the FAILed canary's own pre-registered escalation branch. Prediction-if-true (capacity was the bottleneck): reward stops declining and starts climbing with frontier/eval moving together, roughly matching the MLP sibling's shape. Prediction-if-false (still declining/leg-locking at the bigger width): the pathology is not pure parameter-count capacity -- something about the attention mechanism itself (e.g. history-window framing, positional handling at 100Hz) is broken, and further tf-scale arms should stop pending a dig-in, not be dosed further.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY, same bar as canary-r1: PASS = boots+trains to 2M with no crash/NaN, healthy CUDA fps, and reward/frontier/eval improving together (not just non-declining) -- if PASS, respec full 40M as an acquisition arm. FAIL-same-signature (reward declining quarter over quarter, frontier stuck at b0) = the collapse is not simple under-capacity; stop dosing tf width/layers and flag DIG-IN for an attention-architecture-specific root cause before any further transformer arm. Directional gait not required at 2M.

**verdict**: CANARY FAIL - MECHANISM CONFIRMED, NOT CAPACITY. Result: the 2L/d128/8h/ff256 escalation (matching the proven pre-100Hz tf-r1-hard1 config) reproduces the IDENTICAL decline shape as the 1L/d64 canary despite 4x+ width/depth -- reward quarters -17.7/-192.7/-376.7/-611.9 (canary-r1: -20.4/-197.1/-384.1/-609.4, nearly indistinguishable), ep_rew_mean -739 (r1: -734). Evidence: walkcurr/frontier pinned at 0 through all 4 cert rounds, frontier_pass=0, promotions=0 -- zero real curriculum practice ever unlocked; env/height_err_mm rises 0->104mm (progressive crouch collapse); env/walk_loadslip_ratio rises 0.2->6.17 (2x over the 3.0 cap); env/walk_direction_valid falls 0.95->0.52 with only a partial late uptick to ~0.69-0.73 (still well below start). Why: per this canary's own pre-registered decision tree (PASS=capacity was the bottleneck; FAIL-same-signature=not capacity), width/depth escalation changed nothing about the trajectory shape or final state -- decisively the FAIL-same-signature branch. What's next: per the gate's own text, STOP dosing tf width/layers; the defect needs a root-cause trace (attention-weight/gradient-norm inspection, whether hist64's 0.64s window position handling is broken specifically at 100Hz control cadence) before any further transformer arm. DR-0/joygate reads not required at this 2M canary bar and not waited on (training telemetry alone is fully decisive and matches r1's shape point-for-point). Flagging DIG-IN for the architecture trace.

