# cw-stand-rsi3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-11T02:59:25+00:00

**pod**: hexapod-mjx-train-6

**steps**: 2000000

**parent**: cw-stand-rsi2

**wandb_id**: 0g4ajmcz

**hardware_ready**: False

**hypothesis**: ONE change vs cw-stand-rsi2: reward.rise_score_strip_pen=1 (commit 02183c6). rsi2 verified the pool-restore fix (env/rise_rsi held ~0.5 all run) yet still collapsed to flag-leg. Reachable-optimum arithmetic: score mode stripped the POSITIVE task kernel but left the k_height=100 quadratic PENALTY live, so belly rest under a +111mm command costs -1.2/tick while flag-leg costs only the -0.5/tick rent -- among behaviors a mediocre policy can actually reach, the cheat was the PAID OPTIMUM, and gradient followed it in every arm. With the penalty stripped: belly rest is free, flag-leg pays pure rent (net negative), and the only positive height slope is score income + the feet-gated ref kernel seeded by RSI spawns. A parallel operator session independently hit the same seam from the walk side (walk_height_gate, a808c0d: base k_height outbid by walk income).

**gate**: env/rise_rsi holds ~0.5 (fix regression check); env/rise_feet_factor must STOP collapsing (rsi2: 0.87->0.15; flag-leg is now net-negative so feet should stay down); env/rise_score off the 0.01-0.03 floor and climbing; env/reward_rise_ref >=0.3/tick sustained. Harness at 2M: rise valid_plant from flat starts, RSI off at eval. If feet stay down but score still flatlines, the remaining gap is the warm start's out-of-band height obs (2.2x trained range) -> next arm re-anchors the command band or fresh-inits.

**verdict**: FAIL -- known flag-leg/tripod exploit again, video-identical to rsi2/score1/scoreref1 (three legs held in air the whole episode, 0/6 valid_plant+end_posture det+sto). Tested and REFUTES the stray-k_height-penalty-funds-the-cheat theory (reward.rise_score_strip_pen=1): env/rise_feet_factor still collapsed 0.87->0.15-0.19 on the same curve/timescale as rsi2, rise_score never left 0.01-0.09, reward_rise_ref crashed 0.82->0.05-0.08 within ~20 updates. env/rise_rsi held ~0.53-0.58 (mechanism healthy). Reconciled with operator RISE.md 08-11 read: the SAME feet-factor collapse shape/timescale across 6 reward-different arms means this is not reward-driven -- warm-start OOD drift (108-114mm command band ~2.2x trained range) that no income term can anchor. No further reward/income/RSI coefficient variant queued; two CODE levers now specced: (a) BC anchor in the trainer [operator preferred first], (b) structural height<->foot-contact coupling.

