# cw-dep-vref1-loaded1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-10T15:25:13+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: lud7sng5

**hardware_ready**: no

**hypothesis**: Open-problem-3 named first training arm: dep-line respec with the load-fitted servo model vs its air twin. The loaded actuator gap is quantified (RL_LOG 08-10): 2deg loaded steps settle in ~250-325ms on hardware vs tens of ms in the air fit; loaded velocity ceiling ~48.5 deg/s. bus.servo_params=loaded (landed 08-10, held-out loss 20-40x better than air, liftoff-trace replay RMSE 2.46-2.50deg, delay_ms_pct widens latency DR x0.3-1.9, MJX ring verified) makes training feel that response lag. One variable vs cw-dep-vref1-r1 (identical contract-exact obs, 25deg envelope, warm start, seed). Prediction-if-true: policy retains contract-exact walking under realistic lag (own-cfg gv 12/12, 0 term, vel err within 15% of vref1-r1 on the same eval) - a strictly more hardware-honest attempt-#2 base. Prediction-if-false: gait breaks/terminates under ~106ms-band latency - air-trained bases overfit instant servos and the hardware base needs retraining on loaded params at larger budget (decides the dep-line default). Strongest alternative: trains but goes creep/park to dodge lag - compare videos vs vref1-r1, not just scalars. ASSUMPTION (operator to review): loaded mechanism validated env-level + replay per RL_LOG 08-10; no separate pod-side probe smoke before this arm - watcher checkup + first eval will catch integration breakage.

**gate**: own-cfg det+sto 6/6 @15s gait_valid 12/12, 0 term, vel err within 15% of cw-dep-vref1-r1 on the same eval config; explicit side-by-side video vs vref1-r1; frames watched det

**verdict**: PASS (dig-in, matched-parent control). OBSERVATIONS: own-gate eval (loaded physics, seed 0) det prog 1.09/slip 1.42/fwd 0.81m, sto 1.04/1.65/0.76m, gv 12/12, 0 term; triage had flagged vel_err +40%/slip +60% vs the parent's CLEAN-physics band. Control (frozen cw-dep-vref1-r1 under the IDENTICAL loaded injection+seed, logs/ckpt_eval/cw_dep_vref1_r1_loadedctl): det 1.02/1.44/0.76m, sto 0.92/1.81/0.68m, gv 12/12, 0 term, same sto/4 crater fingerprint (10.15 vs child 10.07); both videos show the same clean six-leg creep, no flag/drag/skate. INTERPRETATION: the delta is the honest physics cost of loaded servo response - it hits the frozen parent identically; the 15% gate was mis-specified against the clean-physics band (same GATE LESSON class as actnoise 08-10 18:28). Child matches-or-beats parent on every median under matched physics (fwd +7% det/+12% sto, sto slip -9%). VERDICT: PASS - training on bus.servo_params=loaded retains the dep-contract walk and slightly improves it under realistic servo lag; loaded params are a viable dep-line training default. hardware-ready: no as-is (walk gate only; joystick/Gate-0 panels not run) - candidate base, air-vs-loaded for attempt #2 is an operator bench decision. HYPOTHESIS: if-true partially confirmed (retention yes; within-15%-of-clean-parent was the wrong yardstick), if-false refuted (no gait break), park/creep-dodge alternative refuted by video.

