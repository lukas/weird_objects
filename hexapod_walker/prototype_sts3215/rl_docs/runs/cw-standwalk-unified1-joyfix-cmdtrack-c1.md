# cw-standwalk-unified1-joyfix-cmdtrack-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-28T16:19:11+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-unified1-mix-long-s0

**wandb_id**: f74ne0cw

**hypothesis**: Plain English: does adding the direct normalized joystick objective (walk_cmd_track_score: +1 at the requested velocity, -1 parked, -2 cross-track, -3 backward, stop charged by speed) on top of the unified recipe's existing walk income sharpen speed/direction tracking without inverting the income ordering (parking/marching must not become competitive)? Operator-proposed (fb_20260828T153912_c528ce); demoted post-audit from 'the missing objective' to one controlled arm, since the 08-28 audit showed the policy already follows commands (fwd dir_err 1.4deg, slip 2.88) and the real gaps are lateral draw coverage and velocity-blind obs. Bank ordering test re-run PASS this cycle (test_joymodes_direct_command_score_orders_exact_direction_first). Warm from long-s0's 16M PASS checkpoint, otherwise identical recipe. Predict-if-true: fwd speed_ratio rises above the parent's 0.475 by 2M with slip and terminations at parent band. Predict-if-false: income shifts to jerky overspeed or march-in-place stalls reappear (this lever made overspeed WORSE in the 08-20 fast-gait lineage — the known risk) — closes the scalar lever for standwalk too. Strongest alternative: no measurable delta at 2M because k=2 is small next to k_walk_course=2+k_walk_prog=2 (dose question, not mechanism).

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. CANARY (mechanism health only): at 2M — training reward not collapsing; DR-0 det walk gait_valid >=5/6 no new sacrificed leg; session terminations <=6/90; probe fwd speed_ratio >=0.40 (parent 0.475 band) and no overspeed inversion (achieved/commanded <=1.25); slip/m in session eval not >1.5x parent median. PASS+delta -> dose bracket next; PASS+no-delta -> record dose-limited; FAIL by ordering inversion -> scalar lever closed for standwalk with this run as evidence.

