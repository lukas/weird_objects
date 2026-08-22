# cw-dep-bcgait4-phasedir1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-22T00:24:31+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-dep-bcgait2-fastbc1

**wandb_id**: 1o4lcds8

**hardware_ready**: False

**hypothesis**: Give the copied fast walking gait an explicit clock and ask it only to change direction, not speed: the policy now sees the gait phase (sin/cos of a clock at the teacher's 0.75 s period, advancing only while a velocity is commanded - implementable identically on the robot's board) as two extra observations, is BC-initialized from a fresh phase-conditioned clone of the native-cadence TripodGait teacher under the full fast servo profile, and trains at ONE fixed speed (0.08 m/s commanded) with a fixed heading per episode. Operator order 08-22 (fb_20260822T000627 + focus note): the copied gait may be brittle because PPO must infer phase from history, and honest direction control at one good speed outranks speed obedience - the three refuted speed-obedience levers are NOT retried (vel:=ref obs contract exactly as the clean fastbc1 parent; no overspeed/heading charges). Preflights THIS cycle at the NEW measured plant (tibia 150 mm, operator commit a4beb8af - first fast-gait arm on measured geometry): teacher grid 0.06/0.08/0.10 x 4 headings x 3 seeds all clean (zero terminations, 169 mm tall, prog 0.68-0.77, slip/m 1.44-2.9; logs/probe_phasedir/teacher_full_*.json, train-0..2); phase clone holdout act err 0.0040 (vs 0.012-0.013 for the blind clones - the phase input disambiguates the cycle), closed-loop clone clean on all 4 headings at 0.08 (prog 0.67-0.80, slip/m 1.56-2.07, zero terminations, all six legs cycling; clone_phase1_c0.08.json, train-1). Prediction-if-true: RL fine-tune stays on the clock manifold (no speedbc1-style fall/tangle regression) and heading error stays <=30 deg across fixed headings. Prediction-if-false: falls/phase-break despite the visible clock = the brittleness is not phase-observability, STOP.

**gate**: At 2M, DR-0: (a) FIXED-HEADING panel - eval_checkpoint det+sto with --cfg-set goal.walk_speed_min_m_s=0.08 goal.walk_speed_max_m_s=0.08 goal.walk_heading_max_rad=3.1416 (random fixed heading per episode, no resample): ZERO falls, gait_valid 6/6, direction_err med <=30 deg, slip/m det <=2.2 / sto <=3.0, height in-band at the new 169 mm plant, roll_tail/drag reported vs fastbc1 (roll_tail 1.1-2.3 deg); (b) standard walk row + pinned-speed row @0.08 forward: zero falls, clean 6-leg. Speed obedience NOT gated (operator: direction over speed; report prog_ratio/speed_mean for the record). PASS -> pre-registered next rung, only then: irregular heading CHANGES at the same fixed speed (goal.walk_cmd_resample_s>0 with jitter/fast blends) per the operator curriculum. FAIL modes: falls/tangle/phase-break under RL = phase input does not keep the clone on-manifold, STOP + operator; clean gait but heading err >30 = direction conditioning failed, STOP + operator. NO DOWNLOAD_ANSWER change from this run under any outcome.

**verdict**: FAIL per pre-registered mode (b) — clean gait, heading obedience lost: 2M PPO fine-tune degraded the phase-conditioned clone on every gated axis vs the MATCHED un-RL'd clone control on the identical fixed-heading panel (dir_err med 35.6->67.3 deg, rear headings collapse to prog 0.01-0.07, speed 0.068->0.139 = the same ~1.7x overspeed attractor as fastbc1, slip/m 1.81->4.17, roll_settled 12/12->5/12) while keeping zero falls / gait_valid 12/12 / tall+clean roll (unlike speedbc1's 34/48 falls — the phase input DID keep RL on the gait manifold). STOP + operator per gate. KEY POSITIVE ARTIFACT: the init clone itself (ppo_goal_cw_bcgait_init_fullprof_phase1.zip, committed) PASSES the direction-first ask at BC level with ZERO RL: fixed 0.08 cmd, uniform random fixed headings incl. rear, prog 0.65-0.76 every episode, slip/m 1.6-2.0, in-band speed 0.068-0.074, roll settled 12/12, full fast profile, NEW measured plant (tibia 150, a4beb8af). Note: direction_err_mean_deg has a ~35 deg floor from tick-level stride sway (the near-perfect clone reads 35.6; pinned-forward child reads 38.7 while beelining 1.58/1.61 m) — the <=30 bar is uncalibrated for this metric; the clone-vs-child DELTA is the honest gate reading. Evidence: logs/probe_phasedir/gate_phasedir1_det + gate_phaseinit_det (train-1), teacher grid teacher_full_c*.json (train-0..2).

