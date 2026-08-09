# cw-stance-endpost-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T00:25:29+00:00

**pod**: hexapod-sweep-s5

**steps**: 4000000

**parent**: ppo_goal_cw_stance_dr10.zip (md5 da1d912a)

**wandb_id**: no0ihywt

**checkpoint**: pod s5: rl_move/sim/policies/ppo_goal_cw_stance_endpost.zip (md5 b78c1b6ab41f51cc2753a44ff828aecb; controller copy /tmp/ckpts)

**hypothesis**: Terminal end-posture pricing (retry of cw-stance-endpost: first attempt died at init, parent ckpt missing on the new s5 pod — fixed, md5 verified da1d912a). The flag ending survives because no term has gradient on an airborne leg during the terminal phase; k_end_posture=5.0 (schedule-gated clearance charge, last ~1.5s, transients untaxed, allowances mirror eval 20/60mm) supplies it, so the planted ending is reachable at INHERITED std (no basin escape after posture2 std runaway). One variable vs cw-stance-posture (same parent/cfg/seed): +k_end_posture. If-true: lower end-posture >=4/6 det or sto (baseline 0/6, worst_clear 256-264mm), heights retained. If-false: worst_clear ~250+mm, -46/episode absorbed -> refutes dense-terminal-gradient; remaining option = belly-rest reference states. Strongest alt: k=5 distorts descent to dodge the window - impossible, window is time-based. Probe: 150k integration clean + local band check. Snapshot 5589bc4.

**gate**: posture-strict harness @ DR 1.0, 6 eps/mode det+sto: lower end-posture >=5/6 sto AND >=4/6 det AND rise/lower height-only >=5/6 both AND hold sto 6/6

**verdict**: FAIL (cycle 18): lower posture 0/6 det + 0/6 sto vs gate >=4/6 / >=5/6; heights clause MET (rise/lower 24/24), hold sto 6/6 MET. Dose-response unique in lineage: flag leg 4 moved 229-264 -> 133-207 mm, leg 2 into allowance (22-48 mm), summed over-allowance ~200 -> ~147 mm; endings HOVER (lower two spear legs ~114/150 mm; rise sprawl 24-64 mm, sto posture regressed 4/6 -> 1/6 outside noise). std 0.193-0.197 no runaway; end_posture charge Q1 -0.640 -> Q4 -0.518, still declining. NOT HARDWARE-READY. Champion unchanged. HYPOTHESIS INCONCLUSIVE -> one consolidate-in-place continuation (c1) with pre-registered plateau slope rule; plateau => belly-rest reference states. Videos: lower_det_0 2cd2d82f, raise_det_0 d7a53139, rise_det_0 308d2472, hold_det_0 9e2e0d2c (250 fr each). Harness footgun recorded: joint_goal default modes = (hold,track,rise); always pass --modes for gate evals.

