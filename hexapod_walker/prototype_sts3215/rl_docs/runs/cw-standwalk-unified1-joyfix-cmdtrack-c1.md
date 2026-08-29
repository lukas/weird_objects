# cw-standwalk-unified1-joyfix-cmdtrack-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-28T16:19:11+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-unified1-mix-long-s0

**wandb_id**: f74ne0cw

**hypothesis**: Plain English: does adding the direct normalized joystick objective (walk_cmd_track_score: +1 at the requested velocity, -1 parked, -2 cross-track, -3 backward, stop charged by speed) on top of the unified recipe's existing walk income sharpen speed/direction tracking without inverting the income ordering (parking/marching must not become competitive)? Operator-proposed (fb_20260828T153912_c528ce); demoted post-audit from 'the missing objective' to one controlled arm, since the 08-28 audit showed the policy already follows commands (fwd dir_err 1.4deg, slip 2.88) and the real gaps are lateral draw coverage and velocity-blind obs. Bank ordering test re-run PASS this cycle (test_joymodes_direct_command_score_orders_exact_direction_first). Warm from long-s0's 16M PASS checkpoint, otherwise identical recipe. Predict-if-true: fwd speed_ratio rises above the parent's 0.475 by 2M with slip and terminations at parent band. Predict-if-false: income shifts to jerky overspeed or march-in-place stalls reappear (this lever made overspeed WORSE in the 08-20 fast-gait lineage — the known risk) — closes the scalar lever for standwalk too. Strongest alternative: no measurable delta at 2M because k=2 is small next to k_walk_course=2+k_walk_prog=2 (dose question, not mechanism).

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. CANARY (mechanism health only): at 2M — training reward not collapsing; DR-0 det walk gait_valid >=5/6 no new sacrificed leg; session terminations <=6/90; probe fwd speed_ratio >=0.40 (parent 0.475 band) and no overspeed inversion (achieved/commanded <=1.25); slip/m in session eval not >1.5x parent median. PASS+delta -> dose bracket next; PASS+no-delta -> record dose-limited; FAIL by ordering inversion -> scalar lever closed for standwalk with this run as evidence.

**verdict**: CANARY FAIL - MECHANISM (4/5 criteria PASS but the decisive one FAILS): adding reward.k_walk_cmd_track=2.0 (raw per-tick command-tracking score, no EMA) on top of the unified1-mix stack, warm from long-s0's 16M PASS checkpoint, does NOT sharpen direction tracking and makes slip materially worse. Session-level (mixedsession, 90 episodes, dr0+owndr+dr0_long): direction_err_med_deg 66.045 vs parent long-s0's own 64.6 baseline (flat/no improvement -- the whole point of the lever), slip_per_m_med 23.233 vs parent 9.17 (2.53x, blows the gate's own <=1.5x-parent cap by a wide margin). The other 4 criteria are fine in isolation (reward recovers Q3->Q4 -783->-272 like every non-failing joyfix sibling; DR-0 det walk gait_valid 6/6 sac []; session terminations 2/90 <=6 cap; probe fwd speed_ratio 0.425 >=0.40 with no overspeed inversion), so this is not a training-instability failure -- it is exactly the hypothesis's own predicted-if-false branch (income shifts to jerky overspeed, matching the 08-20 fast-gait lineage's known risk) landing as measured evidence. Directly answers the k_walk_course root-cause diagnostic's own pre-registered follow-up ('if cmdtrack-c1 also fails to move dir_err, fix k_walk_course itself rather than replace it'): the raw-tick alternative is refuted, not just untried. CLOSES the k_walk_cmd_track scalar lever for standwalk (matches its joystick-track precedent, closed 0/3 there too). Evidence: logs/ckpt_eval/cw_standwalk_unified1_joyfix_cmdtrack_c1_{gate,owncfg,mixedsession}/.

