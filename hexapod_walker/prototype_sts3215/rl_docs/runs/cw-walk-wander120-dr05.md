# cw-walk-wander120-dr05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T20:31:08+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-walk-wander-dr05

**wandb_id**: iai5ea56

**hypothesis**: 2-minute continuous wandering (resample every ~5 s, 15% stops) trains through fatigue-horizon effects 60 s never sees. If-true: gait survives 120 s without drift/degradation (own-cfg gv 12/12, 0 term @120 s). If-false: long-horizon drift appears - endurance needs explicit anti-drift terms, not exposure.

**gate**: own-cfg harness det+sto 6/6 @120 s: gait_valid 12/12, 0 term, det median fwd >=4.8 m; frames watched det

