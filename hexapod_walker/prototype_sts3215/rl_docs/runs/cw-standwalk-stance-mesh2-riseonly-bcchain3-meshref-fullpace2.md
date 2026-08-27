# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-fullpace2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-25T17:22:06+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref

**hypothesis**: Does the rise still need the slowed-down copying schedule now that the demonstration itself is one the motors can afford? The half-pace anchor chain (lookahead 0.25s / min_h 8mm) was dosed and tuned entirely against the OLD infeasible primitive-extracted ref, where slowing the pursuit compensated for a demonstration paced above the servo velocity limit; that dose curve may not transfer to the mesh-native quasi-static ref (feasible at 0.53A open-loop). Exact meshref recipe with ONLY the chain pace reverted to original full-pace values (bc_anchor_lookahead_s 0.25->0.5, bc_anchor_min_h_ahead_mm 8->15; floor>=15mm is also the decouple-grid-proven anti-freeze setting). 2M canary, seed 0, judged against the meshref s0/s1 pair. Prediction-if-true: matches or beats meshref's det 5/6 with same-or-cooler currents — pace was only compensating for ref infeasibility, and the faster chain may even help the still-pinned flat start by pulling harder out of the belly dead-zone. Prediction-if-false: regresses vs meshref (more pins/freezes) — half-pace is a real requirement independent of ref content; keep it for all ref-based arms. (Replaces -fullpace, killed: launch error omitted these cfg overrides; first -fullpace2 attempt aborted on a git snapshot race, retry.)

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition or close a behavior/reward class at this checkpoint. 2M canary, DR-0 rise det+sto n=6+6 (same harness as meshref/slowchain). PASS: det>=5/6 AND sto>=4/6 valid_plant with valid-episode currents at-or-below the meshref band (no valid episode above 2.25A) and over_current terms <=3/12 — full pace transfers; prefer it going forward (one less tuned knob) and re-dose future arms against the new ref. PARTIAL: det 4/6 or a mixed read (e.g. flat start newly unpinned but an rsi regression) — pace interacts with ref content; keep half-pace default, note the subclass finding. FAIL: det<=3/6 or oc terms >3/12 or any all-six-leg freeze (cross-check height_err_end_mm) — half-pace required independent of ref feasibility; closed, keep 0.25s/8mm.

**refused_reason**: hexapod-mjx-train-0 already runs cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-fullpace2 — GPU pods host exactly one run; pick a free GPU pod.

