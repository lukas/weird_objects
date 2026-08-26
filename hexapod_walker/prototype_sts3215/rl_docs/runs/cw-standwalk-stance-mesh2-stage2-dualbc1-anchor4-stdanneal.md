# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor4-stdanneal

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-26T21:36:48+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2

**wandb_id**: u0r0mgov

**hypothesis**: Plain sentence: the dual-core stance anchor's hold/sto failure (total 6/6 hold_min_load termination, unchanged across bc_anchor_coef 3.0 and 6.0 -- anchor2/anchor3 both CANARY PASS on the leak-fix but FAIL on dose) looks exactly like the isolated single-core hold rung's own sto/current-pinned failure, which was NOT fixed by BC-anchor dose either -- it was fixed by annealing away exploration noise (bcanchor3-stdanneal PASS, and again this cycle on stancemix-bcchain3-stdanneal: hold went from hot/marginal to a clean full stand once std was annealed 0->-4.0). Single lever vs anchor2 (the leak-fixed, coef=3.0 recipe -- dose is reverted to 3.0 since anchor3 proved 6.0 buys nothing and cost seed1 some walk/sto stability): add --log-std-anneal-frac 0.5 --log-std-final -4.0, same schedule as the proven isolated-hold fix. Prediction-if-true: hold/sto termination drops from the current total 6/6 toward isolated (<=2/6), current/height stabilize (cur_p95 <1.0A) without walk regressing (still gait_valid >=5/6 det both DR, no anchor1-class catastrophe). Prediction-if-false: hold/sto stays pinned at 6/6 even noise-free -- the stochastic-axis failure in the DUAL-core setting is not an exploration-noise artifact the way it was in isolation (plausible: dual-core adds shared-trunk/value coupling the isolated single-core rung never had), and the next lever must target that coupling directly (e.g. per-mode value heads) rather than another stance-side knob.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition or require mature gait. Same joint hold/rise/lower/walk DR-0 det+sto + own-DR(0.5) panel as anchor2/anchor3. LEAK-STAYS-FIXED PASS if walk shows no anchor1-class catastrophe (det gait_valid >=5/6, prog_ratio >=~0.2, no 5-leg sacrifice, no negative-prog high-slip shuffle) on this checkpoint. STDANNEAL-WORKS if hold/sto termination count drops from the anchor2/anchor3 baseline (total 6/6 both DR) to <=2/6 on at least one DR level without walk regressing. FAIL if hold/sto stays an unchanged ~6/6 term noise-free too (anneal is not the lever in the dual-core setting) or walk regresses toward anchor1's catastrophe.

