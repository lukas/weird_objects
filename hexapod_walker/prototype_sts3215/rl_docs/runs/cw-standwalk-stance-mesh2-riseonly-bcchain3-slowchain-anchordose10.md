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

**verdict**: CANARY FAIL - MECHANISM: anchor dose 10.0 actively hurts rise at 2M. DR-0 gate valid_plant 2/12 (det 0/6, sto 2/6 crouch-only) vs slowchain dose-3.0 parent 5/12; over_current terms 9/12 vs 3/12, all pinned at the 2.64A ceiling; herr_end 30-82mm on flat/bridge/rsi deep starts; strips show the same stalled-splay flat-segment posture. Supervision quality did NOT improve with dose: bc_anchor_loss_rise ends 0.100 (parent plateau ~0.05) and ep_rew quarters fall 8 -> -419 as the anchor penalty dominates return. Dose-response 3.0->10.0 is monotonically DOWN, so dose10 cannot be the promoted arm regardless of anchordose6 (owned by the parallel cycle): if dose6 also fails to beat 5/12, the anchor-dose axis closes alongside pace/budget and rung-9 (mesh-native IK rise ref, or direct edit of rise_ref_belly2plant.npz flat segment) is the only live lever. Evidence: logs/ckpt_eval/cw_standwalk_stance_mesh2_riseonly_bcchain3_slowchain_anchordose10_gate/, W&B yei41azm.

