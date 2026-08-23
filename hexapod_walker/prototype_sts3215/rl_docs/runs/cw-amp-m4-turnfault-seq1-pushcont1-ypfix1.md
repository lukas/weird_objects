# cw-amp-m4-turnfault-seq1-pushcont1-ypfix1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-23T05:13:27+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1

**hypothesis**: Plain English: the push-probability dose sweep (0.25/0.5/0.75/1.0, all vs pushcont1) just closed FLAT -- every dose lands tip err in the same 0.24-0.30 band, so exposure fraction is not the lever. But NONE of those arms ever turned on the already-built, already-banked overshoot-farm fix (reward.yaw_prog_overshoot_decay/yaw_prog_avg_s, landed 08-23 from the solo-turn income audit and separately tested on the DIFFERENT turnpushfault1-style05/clamp-pinned lineage) -- pushcont1 and every pushdose child were trained on the LEGACY yaw pricing that pays overshoot up to 1.25x, the exact mechanism the audit named as the M2 turn-erosion driver before push was ever added. This is the untested cell: does closing the farm (pricing correctness), not exposure dosing, recover fault-first-order turn tracking under push? Single lever vs pushcont1: reward.yaw_prog_overshoot_decay 0.0->1.0, reward.yaw_prog_avg_s 0.0->1.0 (income now peaks at ratio 1.0 and never pays past it). Re-inits from the SAME pre-cheat turnfault-seq1 checkpoint pushcont1 itself used (per the 08-22 init-basin rule -- pushcont1's own --init-from, inherited automatically via respec --from), matched 2M discovery budget.

**gate**: Own-cfg DR-0 floor: gait_valid>=9/12 (pushcont1's own bar, cleared at 10/12). Hand-run eval_yaw (hazards zeroed, matched fast-servo cfg): PASS-clean = tip-left AND tip-right err <=0.20-0.25 (matches fault-only parent's 0.18/0.17, closes the M4/M5 turn+push cell). PARTIAL = measurably better than pushcont1's own 0.2727/0.3029 but still >0.25 (pricing helps, doesn't fully solve -- next lever is the hold/forward income-dominance repricing itself). FLAT = ~0.27-0.30 unchanged (pricing is not the residual driver either; the erosion is a from-scratch composition-capacity limit, not a priceable exploit, and the deferred hold/forward repricing build becomes the last named lever before it needs operator sizing). Also report signed yaw_ratio via probe_walk_income (tip_left/tip_right dirs) to confirm the farm ratio itself moves toward 1.0, not just the |err| scalar.

**refused_reason**: hexapod-mjx-train-0 code marker 24a098717519a425f7991d180305f357991f4f35-dirty != local HEAD 24a098717519a425f7991d180305f357991f4f35 and the delta is not benign-orchestrator-only. Sync first: snapshot.sh --sync hexapod-mjx-train-0 (and snapshot/commit before that if the tree is dirty).

