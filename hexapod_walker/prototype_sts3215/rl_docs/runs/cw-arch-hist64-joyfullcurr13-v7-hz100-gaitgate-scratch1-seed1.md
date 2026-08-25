# cw-arch-hist64-joyfullcurr13-v7-hz100-gaitgate-scratch1-seed1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T04:13:18+00:00

**pod**: hexapod-mjx-train-5

**steps**: 40000000

**parent**: cw-arch-hist64-joyfullcurr13-v7-hz100-gaitgate-scratch1

**wandb_id**: d6ynyx6r

**hypothesis**: Plain English: is the from-scratch walk_gait_gate leg-sacrifice-prevention result (gaitgate-scratch1, still training) a repeatable fix or one-seed luck? Byte-identical V7/100Hz/hist64 recipe with reward.walk_gait_gate=1.0 baked in from step 0, only --seed changes (0->1). Pre-registered alongside scratch1 per operator 08-22 batching guidance. New context (hist64-mesh-acq1 dig-in): on the mesh-default family the over_current trip is only avoidable by a properly CYCLING gait (teacher dwell 0.32s < 0.8s trip), so this grid also reads on the mesh over_current failure mode.

**gate**: PASS: DR-0 det gait_valid >=4/6, no leg pinned near-zero duty, frontier promotes past b0 -- matches scratch1 if scratch1 itself passes. FAIL: {0,2,5}-style (or any 3+ leg) sacrifice signature reproduces on this seed too. Read jointly with scratch1 and seed2: 2-3/3 PASS = robust mechanism; 1/3 or 0/3 = seed-dependent or non-working.

**verdict**: FAIL, worse than the ungated parent AND worse than gaitgate-scratch1 itself -- 2/2 tested gaitgate-scratch* seeds now FAIL, closing the seed-robustness question the ledger pre-registered. Held-out 60s joygate: 48/48 falls (parent scratch1 was 39/48; the ungated pre-gate parent was 12/48), gait_valid_frac 0.167, slip/m med 4.214 (cap 2.9). Mixed term signature (62 tilt_pitch + 34 over_current segment-terminations across the 48-episode panel) with leg 4 sacrificed in the large majority of episodes (76 hits) and leg 3 a distant second (25) -- consistent with the mesh model's real +4mm boot asymmetry on legs 0/4 (per hist64-mesh-acq1 dig-in) layering onto the gaitgate mechanism's known failure to prevent leg-sacrifice cheating. Reward rose the whole run (-677 valley -> +630.8 final quarter) while the gate got catastrophically worse -- textbook 08-21 misalignment, but this is the SECOND independent seed to hit it (scratch1 unsuffixed already FAILed the same way), so per the ledger's own joint-reading rule (2-3/3 PASS=robust, 1/3 or 0/3=seed-dependent/non-working) this closes walk_gait_gate-from-step-0 as a non-working prevention mechanism on mesh, not seed luck. DR-0 gate/own-DR passes are still computing on-pod (control.hz=100 + 60s episodes is slow) -- will land at logs/ckpt_eval/cw_arch_hist64_joyfullcurr13_v7_hz100_gaitgate_scratch1_seed1_{gate,owncfg}/ for a future cycle to read if wanted, but the joygate result alone is decisive and consistent with 2 sibling FAILs, so not worth blocking this verdict on. No further gaitgate-from-scratch seed replicates warranted. Next mesh-family lever is the already-launched cw-arch-hist64-joyfullcurr13-v7-hz100-movecur1 (current-dwell charge) trio -- another cycle's territory.

