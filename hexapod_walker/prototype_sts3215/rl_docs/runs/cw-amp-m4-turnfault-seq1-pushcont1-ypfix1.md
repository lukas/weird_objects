# cw-amp-m4-turnfault-seq1-pushcont1-ypfix1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS-partial

**created**: 2026-08-23T05:17:33+00:00

**pod**: hexapod-mjx-train-9

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1

**wandb_id**: x2a1k1yi

**hypothesis**: Plain English: the push-probability dose sweep (0.25/0.5/0.75/1.0, all vs pushcont1) just closed FLAT -- every dose lands tip err in the same 0.24-0.30 band, so exposure fraction is not the lever. But NONE of those arms ever turned on the already-built, already-banked overshoot-farm fix (reward.yaw_prog_overshoot_decay/yaw_prog_avg_s, landed 08-23 from the solo-turn income audit and separately tested on the DIFFERENT turnpushfault1-style05/clamp-pinned lineage) -- pushcont1 and every pushdose child were trained on the LEGACY yaw pricing that pays overshoot up to 1.25x, the exact mechanism the audit named as the M2 turn-erosion driver before push was ever added. This is the untested cell: does closing the farm (pricing correctness), not exposure dosing, recover fault-first-order turn tracking under push? Single lever vs pushcont1: reward.yaw_prog_overshoot_decay 0.0->1.0, reward.yaw_prog_avg_s 0.0->1.0 (income now peaks at ratio 1.0 and never pays past it). Re-inits from the SAME pre-cheat turnfault-seq1 checkpoint pushcont1 itself used (per the 08-22 init-basin rule -- pushcont1's own --init-from, inherited automatically via respec --from), matched 2M discovery budget.

**gate**: Own-cfg DR-0 floor: gait_valid>=9/12 (pushcont1's own bar, cleared at 10/12). Hand-run eval_yaw (hazards zeroed, matched fast-servo cfg): PASS-clean = tip-left AND tip-right err <=0.20-0.25 (matches fault-only parent's 0.18/0.17, closes the M4/M5 turn+push cell). PARTIAL = measurably better than pushcont1's own 0.2727/0.3029 but still >0.25 (pricing helps, doesn't fully solve -- next lever is the hold/forward income-dominance repricing itself). FLAT = ~0.27-0.30 unchanged (pricing is not the residual driver either; the erosion is a from-scratch composition-capacity limit, not a priceable exploit, and the deferred hold/forward repricing build becomes the last named lever before it needs operator sizing). Also report signed yaw_ratio via probe_walk_income (tip_left/tip_right dirs) to confirm the farm ratio itself moves toward 1.0, not just the |err| scalar.

**verdict**: Repricing the turn-overshoot farm helps the M4 turn+push cell but doesn't close it. Single-lever respec of pushcont1 (reward.yaw_prog_overshoot_decay/yaw_prog_avg_s 0.0->1.0, the already-banked fix from the solo-turn income audit, same 2M budget, same pre-cheat turnfault-seq1 init): hand-run eval_yaw (hazards zeroed, matched fast-servo cfg) reads tip-left/right err 0.2471/0.2553, down from pushcont1's own 0.2727/0.3029 (-9%/-16%) and closing on the fault-only parent's 0.1818/0.1708 ceiling -- but tip-right still clears the 0.25 PASS-clean bar by only 0.005 while tip-left (0.2471) is already inside it, so this lands in the pre-registered PARTIAL branch, not PASS-clean, by a hair. Safety floor improved too: own-cfg DR-0 gate gait_valid 12/12 (pushcont1 was 10/12), zero falls, video-reviewed (det+sto contact sheets) -- clean six-leg cycling with one legitimate carried fault-leg per episode (dr.fault_prob=1.0 baked in), one det episode shows a mid-episode roll spike to 33deg that settles back (tail 5.3deg) rather than a true topple. eval_amp_m5: m5_pass=false as expected -- push/fault sections PASS clean, walk section fails on the already-documented permanent-hazard design tension (q_20260823T0130Z, dr.fault_prob=1.0/ext_push_prob=1.0 baked into training cfg means the walk section can't sample a clean hazard-free episode), yaw section fails the suite's generic 0.10 rad/s bar (not this lineage's own bar, expected). CONCLUSION: pricing-correctness is a real, measurable, but insufficient lever on this axis -- confirms the STATUS banner's escalation call. Per q_20260823T0240Z item (b), the funded next M4 turn+push lever is hold/forward income-dominance repricing (hold-freeze pays more than the honest tip ceiling), not another composition/dose/pricing-key arm on this same recipe. Evidence: logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushcont1_ypfix1_{gate,m5}/.

