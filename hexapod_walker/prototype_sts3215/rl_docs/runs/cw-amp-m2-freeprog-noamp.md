# cw-amp-m2-freeprog-noamp

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-22T13:40:23+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-pilot-noamp

**wandb_id**: ova8jg2e

**hardware_ready**: False

**hypothesis**: Can a from-scratch policy discover real stepping once standing still stops paying? The 40M M2 pilot pair proved the legacy walk reward's optimum is a statue (one leg triad planted, one airborne; ALL reward growth was stand-income). This control re-runs the no-AMP pilot config from scratch with the bank-calibrated freeprog anti-slip pricing (statue nets -238/ep vs honest gait +558; test_slipwalk_stork_statue_is_priced_out PASS) plus the pre-registered branch-(iii) envelope narrowing (speed 0-0.25 m/s, yaw +/-0.5, pure-walk diet). Prediction-if-true: by 2M the det video shows six legs cycling and real travel (median fwd >= 0.10 m/15 s), unlike the statue fingerprint. Prediction-if-false: freeze/stall fingerprint repeats (cw-nobc-slipwalk1-r1 froze at 2M under this pricing at a fixed command) — which makes the style05 twin the decisive arm: pricing alone insufficient, motion-prior gradient required. Strongest alternative: the idle/park charges destabilize early training into falls instead of stepping.

**gate**: Discovery (2M, DR-0 harness walk mode, 6 det + 6 sto episodes, own cfg): PASS = zero terminations majority of episodes AND median fwd travel >= 0.10 m/15 s AND gait_valid >= 4/6 det with video showing all six legs cycling (statue/flag/stilt = FAIL regardless of scalars). Judged as the matched control for cw-amp-m2-freeprog-style05; no SKILLS/champion updates. Statue fingerprint here + stepping in the style05 twin = the first real style-vs-control win and the Wave-1 unlock evidence.

**verdict**: FAIL (informative null, matched control): training reward DECLINED across the whole 2M discovery budget (quarters -82->-461->-858->-869/ep, never rising -- the 08-21 'reward rising' leniency does not apply). Held-out DR-0 gate: 8/12 episodes (4/6 det, 4/6 sto) terminated tilt_pitch/tilt_roll within 1-2s of a stable plant start; forward travel 0.008-0.071 m/15s in ALL 12 episodes, nowhere near the 0.10 m bar; slip 7.2-17.0/m. Video: rapid destabilizing topple from a standing start, not the predicted freeze (cw-nobc-slipwalk1-r1's fingerprint) and not real stepping -- a third, unpredicted failure mode. W&B component trace shows reward_walk_freeprog_pen (the cross-track/backward charge) already near its harsh end (-2.8/tick, ~half its -6 floor) from the FIRST logged step, before any learning -- a fresh high-std (0.367) from-scratch actor's uncoordinated flailing draws near-max freeprog penalty immediately, and 2M steps is not enough for it to learn coordinated directionality before repeatedly toppling. DIG-IN queued (see cw-amp-m2-freeprog-style05 pairing + STATUS.md) to root-cause whether this is fixable with a forced log-std anneal (mirroring the joystick track's phasedir8/9 fix) and/or a freeprog/idle-charge ramp-in, before committing another discovery arm.

