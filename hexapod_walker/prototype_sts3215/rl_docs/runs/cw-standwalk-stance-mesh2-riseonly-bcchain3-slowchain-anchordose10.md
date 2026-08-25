# cw-standwalk-stance-mesh2-riseonly-bcchain3-slowchain-anchordose10

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T16:06:42+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-slowchain

**wandb_id**: yei41azm

**hypothesis**: Sibling of -anchordose6 (same cycle): brackets the anchor-dose axis from the other side. If 6.0 shows a dose-response (PARTIAL/PASS), does 10.0 continue improving or overshoot (anchor term dominating the RL objective, e.g. income/posture gates getting starved -- the exact failure mode the stancemix rung already showed under a different dilution). If 6.0 FAILs flat, this confirms dose-insensitivity at 2x-3.3x the default rather than needing a third point to be sure.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. 2M mechanism canary, same DR-0 rise det+sto n=6+6 harness. Read jointly with -anchordose6: PASS/PARTIAL/FAIL same bar. Joint-FAIL of both dose6 and dose10 (both flat at the stalled-splay signature, anchor loss unmoved) closes the anchor-dose axis for rise, same as pace -- forces rung-9 (mesh-native IK ref) or a direct posture-reward redesign. A monotonic improvement 3.0->6.0->10.0 promotes the higher dose to an 8M acquisition instead.

**verdict**: CANARY FAIL - MECHANISM: 10.0 anchor coef (3.3x default) confirms -anchordose6's read, dose-insensitive between 6.0 and 10.0 -- both land in the same worse-than-slowchain hot-overcurrent signature. DR-0 gate det 1/6 valid_plant, sto 2/6, 9/12 over_current terms (slowchain: 3/6+2/6, 3/12); own-DR(0.2) det 2/6/sto 1/6, 9/12 over_current -- essentially identical numbers to dose6 (det1/sto2/terms9 gate; det2/sto1/terms9 owncfg vs dose6's det1/sto0/terms11), not a monotonic worsening with dose, just a flat plateau at 'worse than 3.0'. Reward trajectory nearly overlays dose6's quarter-for-quarter (8.1/-23.5/-222.4/-418.6 vs 8.2/-23.0/-219.5/-420.6) -- both doses converge to the same bad basin, confirming this is a real dose-insensitive regime change (crossing some threshold between 3.0 and 6.0 that pushes the anchor term into fighting the current-hot penalty) rather than a continued dose-response. CONCLUSION (joint with anchordose6): the anchor-dose-UP axis is CLOSED for rise -- 3.0 remains the best-known dose; supervision strength is not the fix for the deep-start pathology in either direction tried (this cycle's 6.0/10.0 up-dose, or the campaign's implicit floor at 3.0 -- never tested below 3.0 for rise specifically, that remains a small open gap but low-priority given hold/lower's own dose grid already showed 0.5 was too weak). Combined with the pace-dose grid's own exhaustion (quarterchain/eighthchain/slowchain-cont8, verdicted by a concurrent cycle this same window), BOTH pace and anchor-dose axes are now closed for rung-8 rise -- rung-9 (mint a mesh-native rise ref from scripted IK) or a direct posture-reward redesign on the flat/bridge/rsi segment are the remaining levers, per STATUS.md. Evidence: logs/ckpt_eval/cw_standwalk_stance_mesh2_riseonly_bcchain3_slowchain_anchordose10_{gate,owncfg}/, W&B yei41azm.

