# cw-dep-vref1-loaded1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T15:25:13+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: lud7sng5

**hypothesis**: Open-problem-3 named first training arm: dep-line respec with the load-fitted servo model vs its air twin. The loaded actuator gap is quantified (RL_LOG 08-10): 2deg loaded steps settle in ~250-325ms on hardware vs tens of ms in the air fit; loaded velocity ceiling ~48.5 deg/s. bus.servo_params=loaded (landed 08-10, held-out loss 20-40x better than air, liftoff-trace replay RMSE 2.46-2.50deg, delay_ms_pct widens latency DR x0.3-1.9, MJX ring verified) makes training feel that response lag. One variable vs cw-dep-vref1-r1 (identical contract-exact obs, 25deg envelope, warm start, seed). Prediction-if-true: policy retains contract-exact walking under realistic lag (own-cfg gv 12/12, 0 term, vel err within 15% of vref1-r1 on the same eval) - a strictly more hardware-honest attempt-#2 base. Prediction-if-false: gait breaks/terminates under ~106ms-band latency - air-trained bases overfit instant servos and the hardware base needs retraining on loaded params at larger budget (decides the dep-line default). Strongest alternative: trains but goes creep/park to dodge lag - compare videos vs vref1-r1, not just scalars. ASSUMPTION (operator to review): loaded mechanism validated env-level + replay per RL_LOG 08-10; no separate pod-side probe smoke before this arm - watcher checkup + first eval will catch integration breakage.

**gate**: own-cfg det+sto 6/6 @15s gait_valid 12/12, 0 term, vel err within 15% of cw-dep-vref1-r1 on the same eval config; explicit side-by-side video vs vref1-r1; frames watched det

