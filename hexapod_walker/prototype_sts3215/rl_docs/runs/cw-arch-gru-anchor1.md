# cw-arch-gru-anchor1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-11T23:30:44+00:00

**pod**: hexapod-mjx-train-1

**steps**: 10000000

**parent**: cw-arch-gru-bc-ft1

**wandb_id**: brnukxek

**hardware_ready**: False

**hypothesis**: ft1 (default lr) and ft2 (lr/KL closed) both prove PPO fine-tune keeps the GRU walk cheat-free for 10M/2M steps but ERODES the BC-distilled hold (6/6 -> 0/6) and never finds rise — the exact no-gradient signature the stand track solved with action-space BC anchors (loweranchor1). This arm adds the now-recurrent-capable anchor stack to the ft1 recipe unchanged: rise ticks anchored to the state-aligned reference, hold/track to the settled pose, lower to the IK descent, walk to the command-conditioned TripodGait, stratified per-mode quotas so no skill dilutes another (anchormix1 lesson). If action supervision at the rollout hidden states holds stance while PPO polishes gait, one GRU finally walks AND sits/stands — the original operator goal; if hold still erodes under a live anchor, the eroding force is not gradient absence and the next lever is freezing.

**gate**: 10M forensics, det@DR0 gate cfg vs ft1 numbers: PASS if walk retention holds (det gait_valid >=5/6, med prog_ratio >=0.80, no parked-leg fingerprint) AND hold det >=4/6 (ft1: 0/6) AND lower det >=4/6 (ft1: 4/6) AND rise det >=2/6 with >=1 non-flat start (ft1 and every BC student: 0-1/6). FAIL if walk collapses to paddle (anchor competes with gait) OR hold stays <=1/6 with anchor loss converged (supervision reaches the weights but the behavior still erodes -> freeze lever, not anchor rung 2) OR rise stays 0 while train/bc_anchor_loss plateaus high (state-aligned targets unreachable for the GRU -> joint-goal pretrain lever).

**verdict**: FAIL (known exploit, STOP) -- walk collapses into a frozen march-in-place under the walk-tick BC anchor: det gait_valid reads 6/6 (no leg literally sacrificed) but prog_ratio 0.01, speed 0.001 m/s, and the video is pixel-static for the full 15s clip -- zero net travel. This exactly reproduces the twice-already-closed BC-anchor-on-walk-ticks mechanism (cw-walk-gaitbc1, cw-omni-transbc1), now on a GRU. Real partial win: hold det 6/6 and lower det 6/6 (up from ft1's 0/6 and 4/6) and rise det 2/6 with a non-flat start (bridge+crouch) all clear their bars -- the anchor DOES protect stance skills on a recurrent net as hypothesized. Gate requires walk retention too (AND), so overall FAIL; per guardrails a known exploit is a complete verdict, no further forensics.

