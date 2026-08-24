# cw-arch-tf64-joyfullcurr13-v7-hz100-canary2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-24T20:49:27+00:00

**pod**: hexapod-mjx-train-6

**steps**: 2000000

**parent**: cw-arch-tf64-small-joyfullcurr13-v7-hz100-canary-r1

**wandb_id**: rel6d200

**hypothesis**: Plain English: was the small-transformer's 100Hz collapse (reward -20->-609 declining, frontier stuck at b0, leg-0 locked 47/48 joygate episodes) an under-capacity problem or an attention-architecture-specific pathology? The canary-r1 read already isolates the failure away from the 100Hz rate and reward stack (a sibling MLP on the identical stack learns healthy, reward rising to 746). This arm escalates ONLY tf width/layers (1L/d64/4h/ff128 -> 2L/d128/8h/ff256, the exact config that DID learn to walk at 40M pre-100Hz as cw-arch-tf-r1-hard1), same V7 certfreeze recipe, same hist64/control.hz=100 window, same 2M canary budget, per the FAILed canary's own pre-registered escalation branch. Prediction-if-true (capacity was the bottleneck): reward stops declining and starts climbing with frontier/eval moving together, roughly matching the MLP sibling's shape. Prediction-if-false (still declining/leg-locking at the bigger width): the pathology is not pure parameter-count capacity -- something about the attention mechanism itself (e.g. history-window framing, positional handling at 100Hz) is broken, and further tf-scale arms should stop pending a dig-in, not be dosed further.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY, same bar as canary-r1: PASS = boots+trains to 2M with no crash/NaN, healthy CUDA fps, and reward/frontier/eval improving together (not just non-declining) -- if PASS, respec full 40M as an acquisition arm. FAIL-same-signature (reward declining quarter over quarter, frontier stuck at b0) = the collapse is not simple under-capacity; stop dosing tf width/layers and flag DIG-IN for an attention-architecture-specific root cause before any further transformer arm. Directional gait not required at 2M.

