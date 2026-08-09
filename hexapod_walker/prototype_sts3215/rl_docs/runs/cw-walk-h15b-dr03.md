# cw-walk-h15b-dr03

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T02:43:39+00:00

**pod**: hexapod-sweep-long5m

**steps**: 4000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_lowent_h15b.zip

**wandb_id**: bzvup62t

**hypothesis**: DR ladder rung 0.3 REDO on verified code (cw-walk-lowent-dr03 was invalid: stale pod code dropped the step-event package). The h15b champion gait survives DR 0.3 introduced as a single rung - moderate-DR fine-tuning robustifies rather than destroys. ONE variable vs h15b (md5 d0a12a94): --no-dr to --dr-scale 0.3. If-true: 15s DR0.3 harness matches h15b DR0 baseline within the 1-2 ep band (fwd>=0.40m >=10/12, gait_valid >=11/12 with any invalid ep the known park class, 0 terminations, det slip mean <=1.0 vs parent 0.912) AND DR0 retention shows no erosion outside noise (parent: fwd 10/12, gv 11/12, 5s det slip 0.584). If-false: (a) gv<=9/12, terminations, or a NEW failure class on camera at DR0.3 - rung too big, drop to 0.15; (b) DR0.3 fine but DR0 retention erodes - warm-start interference. Alt: DR-slop pass (noise-robust skating) - distinguished by det slip + frames at both DRs. Snapshot 9c3bbae.

**gate**: 15s DR0.3 harness (own DR, own cfg) 6eps/mode det AND sto: fwd >=0.40m >=10/12 AND gait_valid >=11/12 (any invalid = known park class only, on camera) AND 0 terminations AND det slip mean <=1.0; DR0 retention 15s inside 1-2 ep noise of parent (fwd 10/12, gv 11/12) and 5s det slip <=0.93; frames watched pathology-first

**verdict**: PASS on the recorded gate (first walk-gate PASS at DR>0): DR0.3 15s fwd 12/12>=0.40, gv 12/12, 0 term, det slip 0.899<=1.0; DR0 retention equals parent (fwd 10/12, gv 11/12, det slip 0.916 vs 0.912); 5s slip clause met but distance-confounded (slip/m 1.82 vs parent 1.53, slow starts). HYPOTHESIS: supported on the letter, refuted on the spirit - named-baseline probes show untrained h15b passes the same gate at DR0.3 AND 0.6, and both ckpts fail DR1.0 only on det slip (1.008-1.061 vs <=1.0) = the skating defect, not DR. Rung was VACUOUS; DR-ladder training arms closed. NOT HARDWARE-READY (1/6 DR0 sto park, slip/m 1.4-1.8). Champion unchanged (h15b d0a12a94); dr03 worse outside noise on 5s det fwd (0.180 vs 0.382). Evals: logs/ckpt_eval/cw_walk_h15b_dr03_ev_*. All 12 DR0.3 strips watched; W&B bzvup62t finished 16.02M cum.

