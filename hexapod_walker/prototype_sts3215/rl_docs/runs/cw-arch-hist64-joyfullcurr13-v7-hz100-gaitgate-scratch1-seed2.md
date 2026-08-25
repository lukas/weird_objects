# cw-arch-hist64-joyfullcurr13-v7-hz100-gaitgate-scratch1-seed2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T04:16:10+00:00

**pod**: hexapod-mjx-train-8

**steps**: 40000000

**parent**: cw-arch-hist64-joyfullcurr13-v7-hz100-gaitgate-scratch1

**wandb_id**: 23i9tpjm

**hypothesis**: Plain English: is the from-scratch walk_gait_gate leg-sacrifice-prevention result (gaitgate-scratch1) a repeatable fix or one-seed luck? Byte-identical V7/100Hz/hist64 recipe, reward.walk_gait_gate=1.0 from step 0, only --seed changes (0->2). Second leg of the seed-robustness grid alongside seed1 (now running on train-5); re-launched directly after 3 backlog-drain REFUSED attempts during a moving-HEAD window (multiple concurrent cycles snapshotting at once) -- code marker now clean (c8726f9f).

**gate**: PASS: DR-0 det gait_valid >=4/6, no leg pinned near-zero duty, frontier promotes past b0. FAIL: {0,2,5}-style (or any 3+ leg) sacrifice signature reproduces on this seed too. Read jointly with scratch1 and seed1: 2-3/3 PASS = robust mechanism; 1/3 or 0/3 PASS = seed-dependent or non-working.

**verdict**: Third and final seed of the walk_gait_gate-from-scratch trio (reward.walk_gait_gate baked in from step 0, mesh family, 100Hz, 40M) -- confirms the STATUS.md's already-closed joint read (2/3 already FAIL) with a 3rd independent FAIL, no surprises. DR0 gate: 24/24 episodes terminated, ALL over_current (Imax pinned at the saturation ceiling), gait_valid 20/24. Held-out 60s joygate: 39/48 falls (cap <=2), slip/m med 3.439 (cap 2.9), dir_err med 46.87deg (allow <=40), gait_valid_frac 0.667 -- same magnitude as the base scratch1 read (39/48) and clearly better than -seed1 48/48, but still a hard FAIL on every clause. Video: rigid tripod-lock, rear legs plant stiff and nose the body up/forward while never releasing into six-leg cycling -- same rigid-lock/over_current signature as both siblings, not a new pathology. TALLY: 3/3 seeds of walk_gait_gate-from-scratch now FAIL -- reconfirms this STATUS's existing verdict that a reward-side anti-sacrifice gate baked in from step 0 does NOT fix the mesh-family leg-sacrifice/over_current basin. No further action needed on this lever; recorded for completeness/closure of the 3rd seed slot. Evidence: logs/ckpt_eval/cw_arch_hist64_joyfullcurr13_v7_hz100_gaitgate_scratch1_seed2_{gate,joygate}/, W&B 23i9tpjm.

