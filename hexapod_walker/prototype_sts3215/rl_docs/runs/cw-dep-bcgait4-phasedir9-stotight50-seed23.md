# cw-dep-bcgait4-phasedir9-stotight50-seed23

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-22T18:50:41+00:00

**pod**: hexapod-mjx-train-2

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-stotight50

**wandb_id**: 53ylwzcb

**hypothesis**: Plain English: the -5.0 noise floor widened seed17's gate margins but HURT seed13 (slip 2.407->2.63), so the dose ladder may be seed17-specific -- this arm tests transfer on seed23, the passer whose own-DR-alone dir median (40.36 at -4.5) is the thinnest named hardening gap. Single change vs the stotight50 PASS: seed 17 -> 23 (identical -5.0 recipe, fresh reinit per the lineage rule). Prediction-if-true: joygate pass=true with dir err below seed23@-4.5's 39.4 and own-DR dir comfortably under 40 (deep floor closes the gap; ladder transfers to some non-17 basins). Prediction-if-false: margins at or worse than seed23@-4.5 (ladder is seed17-only; hardening via dose is closed and seed13@-4.5 stays champion with no further dose arms).

**gate**: eval_joystick_gate 60s randomized session (stress_mix, held-out seed base 90000, DR-0 + own-DR 0.35, det+sto, n=12/pass): PASS = evaluator pass=true AND combined dir < seed23@-4.5's 39.4 AND own-DR-alone dir <= 40 (gap closed). INFORMATIVE = pass=true but dir/slip within noise of or worse than seed23@-4.5 (no transfer; dose hardening closed for seed23). FAIL = pass=false (dose flips the basin).

**verdict**: The -5.0 noise-floor dose transfers to seed23 and closes its named hardening gap. 60s joystick DONE-gate: pass=true, 0 falls/48, gait 48/48, combined slip 2.543/dir 35.31 vs seed23@-4.5's dir 39.4; own-DR-alone dir 36.94 vs the 40.36 gap the arm was launched to close (bar was <=40 — cleared by 3.4 deg). No sacrificed legs (sacrificed_frac all 0), duty 0.44-0.555, contact sheet shows clean six-leg cycling. Det DR-0 15s prog 0.69/slip 1.80 — no det trade. Reward rose all run and converged (quarters -136/90/494/459, final std 0.0067) — reward and gate agree. Meets the pre-registered PASS bar exactly. Dir 35.31 is second-best in family (behind stotight55's 34.97); slip 2.543 still behind champion seed13@-4.5 (2.407), so champion candidate unchanged, but the dose ladder is NOT purely a seed17 phenomenon — it transfers to the seed whose gap was direction-shaped. Next: seed29 twin read (slip-shaped gap) decides whether transfer tracks the gap type.

