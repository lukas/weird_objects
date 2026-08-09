# cw-pose-track

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T13:06:12+00:00

**pod**: hexapod-mjx-train-2

**steps**: 10000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_anchorgate.zip

**wandb_id**: p987r507

**hardware_ready**: no

**hypothesis**: Wishlist #18 [READY] body-pose control, pulled because steering-scope arms are stopped (c39 fdiag consequence), current-economy arms are operator-blocked, and remaining walk arms await in-flight verdicts. One variable off champion 35234ddc: goal-mix walk=1.0 -> walk=0.4,hold=0.2,lean=0.2,track=0.2 (modes already exist in the task; no new mechanism). Champion lineage currently scores track_err ~3.1 deg in trainer evals (walk-only training let pose skills sit). If-true: dedicated mix drops track/lean tracking error to <=2.0 deg det+sto with walk retention intact -> body pose becomes a runtime command like rise/lower. If-false: track err stays ~3 deg or walk erodes -> pose control needs its own line (height-ref approach) instead of a mix. Strongest alternative: mix dilution erodes walk without improving pose (both gates fail). Multi-skill arm: canaries ON per standing rules (rise/lower protected).

**gate**: DR0 harness 15s own-cfg: track+lean modes det+sto track_err_deg mean <=2.0 (baseline 3.1), survived 12/12, 0 term; walk retention det 6 eps gait_valid 6/6 prog_ratio 0.75-1.25 slip/m no worse than champion band; rise/lower canaries no regression; frames watched det for quiet pose hold (no jitter/hover)

**verdict**: FAIL. 10M with a dedicated 0.6 pose mix (hold/lean/track) moved track/lean tracking err from baseline 3.1° to det 2.74°(track)/3.37°(lean), sto 3.2/3.45 — gate ≤2.0 unmet and the delta is inside eval noise → no evidence the mix helps. Frames: pose modes are not a quiet hold — feet keep fidgeting/stepping. Walk retention OK-ish (gv 12/12, 0 term, slip/m det med 1.29 just above champion band; two slow-command overspeed eps prog 1.76/2.22). Pose precision is not practice-limited; parks behind the same pricing/actuation roots. No requeue.

