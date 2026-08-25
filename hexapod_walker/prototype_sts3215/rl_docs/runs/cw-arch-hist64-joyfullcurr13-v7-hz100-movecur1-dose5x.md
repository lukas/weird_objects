# cw-arch-hist64-joyfullcurr13-v7-hz100-movecur1-dose5x

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-25T08:48:24+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-arch-hist64-joyfullcurr13-v7-hz100-movecur1

**wandb_id**: ff5xgy92

**hypothesis**: Plain English: all 3 movecur1 arms at k_walk_move_current=2.0 (MLP-alone, MLP+gait_gate, transformer) FAILed identically, every one pinned at cur_max_a=2.64A (the safety-trip edge) regardless of charge -- before closing the whole current-dwell-charge MECHANISM CLASS (not just this dose) as a fix for the mesh-family over_current/leg-sacrifice exploit, this is a cheap 2M bound-the-mechanism canary at 5x the dose (k=10.0, everything else byte-identical to the original movecur1 2M base) to see whether the Imax=2.64A signature can be moved AT ALL by pricing pressure, or whether it is structurally unreachable by this reward term (e.g. actuator saturation dominating the pricing gradient) regardless of magnitude.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. PASS: cur_max_a measurably below 2.64A (not pinned at the safety-trip edge) at 2M, even if walking itself is still generic-early-valley -- proves the mechanism CAN bite at higher dose, worth a full acquisition-budget arm at this dose. FAIL: cur_max_a still pinned ~2.64A / over_current still fires every episode -- the mechanism is dose-insensitive, not just underdosed at k=2.0, and the current-dwell-charge class is fully closed regardless of magnitude (structural, not a pricing-magnitude problem) -- do not iterate this dose axis further, the standwalk-style teacher-distillation route becomes the only named next lever for this exploit.

**verdict**: CANARY PASS: 5x dose (k_walk_move_current 2.0->10.0, single lever off the movecur1 2M base, everything else byte-identical) moves the previously dose-insensitive cur_max_a signature well off the 2.64A pinned safety-trip edge -- exactly the gate's own pre-registered PASS branch. Evidence: (1) W&B env/walk_move_current_max_a falls from a 2.64A plateau (steps 344k-540k) to 0.37-0.43A by ~900k-1030k and only partially re-rises to 1.36-1.9A by 2M, never re-pinning; terminations/over_current collapses from 1587/440 in the first two report windows to single digits (2-9) for the remaining ~24 windows. (2) Fresh DR-0 deterministic gate eval (own-cfg, held-out, n=6): walk/det mode is CLEAN on the mechanism axis -- cur_max_a 2.17-2.29A every episode, 0/6 terminated, 0/6 over_current (was 100% pinned-2.64A/terminated at k=2.0 on this exact recipe family). Noisier modes (sto, startjitter) still hit 2.64A and over_current on some episodes (not all -- 1.97/2.19A appear too), a real but partial dose response, not a clean full close. Video (walk_det_0-5, frame strips): robot holds a static all-six-legs-sacrificed crouch, zero translation -- the 'generic-early-valley'/park-not-walk look the gate explicitly says is fine to ignore at this budget. Why this matters: it directly refutes the alternative branch (mechanism dose-insensitive/structurally unreachable) that the movecur1 trio's FAIL at k=2.0 had left open, and answers the fork the previous cycle flagged DIG-IN instead of deciding same-cycle. Own-DR(0.5) full pass and joygate were also prestaged; joygate (60s held-out, judged on skill not mechanism, irrelevant to this canary's own gate text) reads FAIL as expected for a 2M mechanism-only run (10/48 falls, sacrificed_frac ~80%, gait_valid_frac 0.083) -- not a contradiction, this run was never meant to pass it. Own-DR gate eval was still computing on-pod at verdict time; DR-0 alone already decides this canary per its own pre-registered branch. Next: per the gate's own PASS clause, fund a full acquisition-budget continuation (matching the k=2.0 trio's own 38-40M precedent) at this same dose to see whether real walking (not just avoided-current parking) emerges once cmd_prog actually moves.

