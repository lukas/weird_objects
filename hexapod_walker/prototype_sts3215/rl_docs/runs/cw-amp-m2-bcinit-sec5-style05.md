# cw-amp-m2-bcinit-sec5-style05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T20:02:37+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-sec5-taskB

**wandb_id**: uhjhj3qm

**hypothesis**: Plain English: does starting from a policy that already walks (the verified scripted-tripod BC clone, ppo_goal_cw_bcgait_init_fullprof_phase1) let the AMP brief's minimal section-5 reward + AMP style KEEP and improve real six-leg locomotion, instead of the crouch-statue basin every from-scratch M2 arm (sec5 grid FINAL 0/4: taskA/B/C + noamp, all statue, crouch height_err 59->85mm in Q1) has landed in? Executes the sec5 grid's own pre-registered prediction-if-false branch ('task restructuring or BC-pretrain phase becomes the next real lever') and brief sec4.3's explicit allowance of the scripted gait 'as an initialization only'. Config: sec5 minimal reward keys verbatim (AMP_MINIMAL_OVERRIDES bank already PASS, 4/4 + full bank green 08-22; term_penalty=400, all SLIPWALK keys zeroed) + the clone lineage's proven obs/env envelope (phase obs, body_vel=2, fast servo profile, stress_mix commands at fixed 0.08 — the exact cmdmix45-seed29 env that loads this init cleanly; yaw-cmd obs and asym-critic dropped for obs/critic compatibility with the clone) + warm-log-std -2.0 to protect the init from noise-bombing; task/style 0.5/0.5 (taskB blend), teacher_v2 lib, seed 7 (grid seed), DR-0, 2M discovery. Prediction-if-true: policy keeps walking — det fwd >=0.10m/15s with cyclic six-leg gait_valid, no crouch collapse, and style/disc metrics give the first AMP measurement on a genuinely locomoting actor. Prediction-if-false: even a walking init collapses into the crouch statue — the sec5 reward itself destroys locomotion, closing init-alone and making task restructuring (height/upright/termination pricing) the next lever. Twin: cw-amp-m2-bcinit-sec5-noamp (identical, zero AMP flags) isolates what style adds on a walking actor.

**gate**: Discovery (2M, judged on det video + DR-0 gate harness + amp scalars, NOT the joystick DONE gate; read JOINTLY with bcinit-sec5-noamp twin). INFORMATIVE-PASS = det video shows sustained cyclic six-leg walking with net fwd travel >=0.10m/15s at the DR-0 gate, no crouch collapse (env/height_err_mm stays near init level, not 59->85mm), disc unsaturated. FAIL-collapse = gait degrades toward statue/crouch (gait_valid falling, fwd <0.10m, height_err climbing) — the sec5 reward destroys even a walking policy; init closed as insufficient, task restructuring becomes the lever. Style read: this arm vs noamp twin on gait_valid/slip/visual naturalness = first controlled style measurement on a locomoting actor; within-noise twin = style adds nothing even when locomotion exists.

