# cw-stance-posture2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-08T23:05:10+00:00

**pod**: hexapod-sweep-s5

**steps**: 4000000

**parent**: ppo_goal_cw_stance_posture.zip (md5 7c2ab2f5)

**wandb_id**: 8qrvip34

**hypothesis**: Pre-registered if-false branch of cw-stance-posture: the posture-priced landscape pays a planted leg 4 at the ENDPOINT (park tripod pays ~0.25/step evenness, cycle 12 measurement) but has zero path gradient before first contact (load_even/support_margin blind to unloaded airborne legs; stance_clearance excludes rise/lower/raise); the flag ending is a local optimum unreachable at std 0.195. Re-opened exploration (std 1.0 audited, ent 0.01) lets sampled excursions reach contact, then the dense terms take over. One variable vs cw-stance-posture: exploration noise; reward config identical (NOT coefficient iteration). If-true: lower end-posture >=5/6 sto (leg 4 plants), rise crouch improves, heights recover by run end. If-false: std re-anneals with leg 4 still parked -> refutes exploration-suffices; remaining options are structural (terminal-posture pricing / belly-rest reference), designed not tuned. Risk bounded by canaries + auto-stop (four protected groups). Snapshot e85a290.

**gate**: posture-strict harness @ DR 1.0, 6 eps/mode det+sto: lower end-posture >=5/6 sto AND >=4/6 det AND rise/lower height-only >=5/6 both passes AND hold sto 6/6

**verdict**: FAIL. std ran away 1.0->2.29 (never re-annealed); flag pathology SPREAD (two leg pairs end airborne). Gate: lower posture 0/6 all passes (need >=5/6 sto), hold sto 4/6 vs 6/6 required; hold det 2/6 vs parent 5/6, rise sto 0/6 vs parent 4/6 (outside noise). Heights retained (lower height_err<=16mm) - failure is end-posture only. NOT HARDWARE-READY. HYPOTHESIS REFUTED (exploration-suffices); warm-start + flat ent 0.01 = std runaway, 2-for-2 with hist8. Champion unchanged. Do not warm-start from this ckpt. Eval: logs/ckpt_eval/cw_stance_posture2_4M_gate2, videos reviewed md5 bbc341a2/12e1646d/24bc0b82/961c85ca.

