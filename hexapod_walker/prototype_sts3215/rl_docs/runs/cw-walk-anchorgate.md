# cw-walk-anchorgate

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T08:25:31+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_parkstart_mjx.zip

**wandb_id**: jiwhh0qd

**hypothesis**: Paddling persists because velocity income is fully collectible while stance feet sweep: additive charging is refuted 2x (kernel-gating c24 cut park income 5x, behavior unchanged; effort c29, 18%-of-income charge paid for 20M steps) and a timing reference is refuted (phase prior c30: agreement locked 0.47->0.93, slip unmoved/worse). This arm makes velocity income CONDITIONAL on anchored stance by construction (operator 0-c.2 / step0 mandate): r_walk and positive r_prog are multiplied per tick by the anchored fraction of loaded feet (foot anchored = within 10 mm of its own touchdown point; walk_anchor_gate=1.0, anchor_tol_mm=10). Scale audit (c30): champion paddling collects only 0.53-0.70 of income at tol=10 (creep ~24 mm/stance), an anchored gait ~1.0 - a ~2x-the-effort-arm stake, dense and multiplicative. If-true: walk_anchor_frac climbs from ~0.5-0.7 toward >0.85 EARNED, DR1.0 agg slip/m det<=1.0 AND sto<=1.0 (champion 1.543/1.295), speed stays 0.043-0.057 band, gait retained. If-false shapes: (b) anchor_frac flat with income just forfeited => anchored stance not locally reachable at a 30-47% income stake => local-gradient hypothesis dead, escalate to fresh-init step0-style walk WITH the gate from step 0 or distillation (plan item 0); (c) anchor_frac >0.85 without slip falling => tolerance mis-set or metric mismatch vs harness slip - ONE audit-driven tolerance correction pre-registered, else rung closed; (d) collapse below speed band / gait degrades (DR0 det fwd mean <0.45, gv failures) => income starvation, kill/quarantine, same escalation as (b). Strongest alternative: slip/m falls via slower speed rather than anchoring - excluded by the speed-band check and fwd distances holding. Exploit watch: unload-sweep (feet under contact threshold while sweeping are uncounted) shows as duty drop + swing spike with slip still high - but truly unloaded feet cannot transmit propulsive friction, so beating the gate that way IS stepping.

**gate**: DR1.0 harness 15s own-cfg 6+6: agg slip/m det<=1.0 AND sto<=1.0, gv 12/12, 0 term; DR0 harness 15s: det fwd mean >=0.55, gv 12/12, fwd-hemisphere sto fwd>=0.40 5/5 (backward draw sto[5] recorded-excluded pending operator ruling); frames: stance anchoring (feet stationary in world during stance) vs champion paddling; W&B walk_anchor_frac trend read against if-false shapes (b)/(c)/(d)

**verdict**: FAIL on gate (DR1.0 agg slip/m det 1.240 / sto 1.245 vs <=1.0) BUT first lever of four to move slip: det -20% vs champion parkstart_mjx 1.543, 4/6 eps below champion per-ep min, at unchanged speed and higher fwd (0.681 vs 0.618); sto 1.245 vs 1.295 no evidence; DR0 retention passes (det fwd 0.736, gv 24/24, 0 term, fwd-hemi sto 5/5). anchor_frac 0.767->0.837 monotone still climbing, income earned (reward_walk ~1.2), current flat. No if-false shape fired; hypothesis INCONCLUSIVE; NOT HARDWARE-READY (slip 1.24 m/m). CHAMPION UPDATED to this ckpt. Continuation c1 launched.

