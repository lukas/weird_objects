# cw-walk-phaseprior

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T07:33:30+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_parkstart_mjx.zip

**wandb_id**: ia5x7piz

**checkpoint**: rl_move/sim/policies/ppo_goal_cw_walk_phaseprior.zip

**hypothesis**: Paddling persists because the objective gives the policy no coordination REFERENCE to reorganize toward: pure pricing is refuted twice (cycle 24 kernel-gating: park income cut 5x, behavior unchanged; cycle 29 effort: 18%-of-income charge paid for 20M steps, current/drag/slip all flat). An observable 0.4 Hz tripod clock (+2 obs via MJX transplant, parent-bit-identical at init) plus contact-agreement income (k_phase_contact=1.0: max +1.0/tick vs +2.7/tick kernel+prog; ~0 at champion's current behavior per scale audit - champion cadence is IRREGULAR, best-offset agreement 0.53-0.84 by seed at 0.4 Hz) makes anchored, clock-locked stance gradient-reachable. NOT cycle-12's refuted basin-escape arm: this parent HAS a genuine six-leg gait. If-true: env/phase_agreement climbs >0.7 AND DR1.0 agg slip/m det<=1.0 AND sto<=1.0 (champion baseline 1.543/1.295) with gait retained. If-false shapes: (a) agreement climbs >0.7 but slip stays >=1.2 - timing lock is orthogonal to stance anchoring, phase rung closed, review's remaining rung is dense decomposition (operator gate rulings may reorder); (b) agreement flat ~0.5 - the prior cannot engage a warm gait (income insufficient vs velocity income risk), phase rung closed same way; (c) gait degrades (DR0 det fwd mean <0.45, gv failures, or tripod-park duty signature) - the cycle-12 pathology returned on a warm parent, kill/quarantine. Strongest alternative: slip/m falls via slower speed rather than anchoring - excluded by checking speed stays 0.045-0.057 band and fwd distances hold.

**gate**: DR1.0 harness 15s own-cfg 6+6: agg slip/m det<=1.0 AND sto<=1.0, gv 12/12, 0 term; DR0 harness 15s: det fwd mean >=0.55, gv 12/12, fwd-hemisphere sto fwd>=0.40 5/5 (backward draw sto[5] recorded-excluded pending operator ruling); frames: stance anchoring + cadence regularity vs champion paddling; W&B env/phase_agreement trend read against if-false shapes

**verdict**: FAIL (gate: DR1.0 agg slip/m det 1.786 / sto 1.523 vs <=1.0; gv 12/12 and 0-term met; DR0 retention met). HYPOTHESIS REFUTED on pre-registered if-false branch (a): phase_agreement locked 0.47->0.93 with clean tripod duty at eval, slip unmoved (worse +16-18% vs champion 1.543/1.295, at/just beyond panel noise, consistent direction). Timing orthogonal to anchoring; phase rung CLOSED. NOT HARDWARE-READY: paddling unchanged. Champion unchanged (parkstart_mjx md5 01d9ab60). Frames: dr10 det_0/sto_0/sto_5 + 15s det_0/sto_0/sto_5 watched (md5s in RL_LOG) + dense filmstrip vs effort baseline.

