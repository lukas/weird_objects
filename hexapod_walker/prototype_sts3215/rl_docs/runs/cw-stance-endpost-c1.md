# cw-stance-endpost-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T01:47:39+00:00

**pod**: hexapod-sweep-s3

**steps**: 4000000

**parent**: ppo_goal_cw_stance_endpost.zip (md5 b78c1b6ab41f51cc2753a44ff828aecb)

**wandb_id**: v5t38fee

**hypothesis**: Consolidate-in-place continuation of cw-stance-endpost-r1 (one variable: +4M steps). The r1 hover is optimization-incomplete, not an airborne equilibrium: the terminal charge's descent (r1 Q1 -0.640 -> Q4 -0.518 per charged tick) continues and reaches planted endings. If-true: end_posture improves segment-over-segment by >= r1's 0.12 delta AND lower det summed over-allowance <80 mm (r1 ~147) AND lower posture >=1/6 any pass. If-false: charge plateaus (c1 last-quarter within 0.05 of c1 first-quarter) with per-leg clearances at r1 values (leg0 ~114, leg4 ~150 mm) -> refutes reachability -> belly-rest reference states, no third pricing arm. Strongest alt: slow nonzero descent that neither plateaus nor plants -> slope rule (<0.12/4M at 8M cum = plateaued). Rise-regression watch: rise HEIGHTS <5/6 anywhere = stop the line. Canaries ON.

**gate**: posture-strict harness @ DR 1.0, 6 eps/mode det+sto (explicit --modes): lower end-posture >=5/6 sto AND >=4/6 det AND rise/lower height-only >=5/6 both AND hold sto 6/6

**verdict**: IMPORT-ERA STUB (2026-08-09 shadow-ledger import): this run predates the verdict-field discipline; its science, if any survived, is in archive/ reviews of the 08-09 era. No verdict was recorded at the time and none is reconstructed now — do not count this entry as an open triage item.

