# cw-mt-c2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-12T20:31:42+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-mt-c1

**wandb_id**: kvbcuqox

**hardware_ready**: False

**hypothesis**: The broadest command diet: walk, stop, turn AND sidestep commands for one fresh policy — re-run of the broader generalist (narrow-generalist commands plus heading up to 0.34 rad, i.e. lateral velocity up to ~0.02 m/s) at the 20M budget the recipe needs, after the 2M wave proved under-budget for every arm. Prediction-if-true: cw-mt-b2's full gate passes AND lateral probes respond sign-correctly AND the three zero-shot interpolation probes complete without falls. Prediction-if-false(acquisition): forward/yaw arrive but lateral never does — command-space breadth has a hard edge at this capacity. Strongest alternative: diversity slows gait emergence enough that 20M is under-budget only for THIS arm — visible as b2 walking while c2 creeps.

**gate**: cw-mt-b2's full gate PLUS: sign-correct lateral response on +-0.02 m/s vy probes, and the three MULTITASK.md zero-shot interpolation probes (vx=0.037 wz=0.07; vx=0.025 vy=0.012; vx=0.05 wz=-0.11) complete without falls. FAIL(budget)/FAIL(acquisition) labels binding per MULTITASK.md.

**verdict**: FAIL(acquisition-failure) per MULTITASK.md labels, worse than budget or shortfall — with the broadest command distribution (C), the policy never reaches a valid six-leg gait at the SAME 20M budget where the specialist (a2) passed cleanly. Leg index 2 is a true flag leg in every single episode across both eval configs (duty_cycle 0.01-0.03 vs the other five legs 0.35-0.69) and the robot falls (term_reason=tilt_pitch, roll_class=fell) in 10/12 gate episodes and 9/12 own-cfg(DR0.2) episodes. gait_valid: gate det 0/6, gate sto 2/6, owncfg det 1/6, owncfg sto 2/6 (vs a2 6/6 everywhere, 0 terms). prog med numbers look nontrivial (gate det 1.01, owncfg det 0.96) only because progress is measured on the drag-then-fall trajectory, not a real walk cycle — video confirms: robot holds a near-normal stance on 5 legs with leg 2 splayed out rigid, drags forward briefly, then topples nose-first around frame 8-10 of every clip. Never reaches the point of testing the lateral/interpolation probes in its own gate (moot — it cannot clear plain forward without falling). Establishes the wave-1 monotone story at MATCHED 20M budget: a2 (specialist) clean pass -> b2 (narrow generalist) FAIL(acquisition-shortfall), real gait but underpowered, no falls -> c2 (broad generalist) FAIL(acquisition-failure), flag-leg + falls, no valid gait at all. Command-width interference is real and monotonic, not a 2M-only budget artifact.

