# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-flatmix70

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T17:41:20+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref

**wandb_id**: jkpuza6s

**hypothesis**: Does making the robot practice standing up from lying completely flat much more often teach it to tuck its splayed front legs under the body instead of pressing up on them until the motors overload? Flat-start rise is now the SOLE structural failure of the mesh-native-ref recipe: it fails identically (front legs never tuck, radial press-up pinned at 2.64A, h_err_end 24-28mm) at 2M in both canary seeds AND in both full-8M seed-0 replicates (acq8m + the dup-killed meshref-8m, which finished 8.06M before its kill and scored exactly canary level — proving the 2M->8M budget delta is within same-seed run-to-run noise). Yet flat starts get only ~17.5% of training exposure (rise_rsi_frac 0.5 x rise_flat_frac 0.35). Single lever, config-only, the mechanism the rung-9 grid pre-registered: within the non-rsi half, reweight flat/partial/crouch 0.35/0.40/0.25 -> 0.70/0.20/0.10, doubling flat exposure to ~35%; rsi/crouch curricula and everything else identical to the meshref 2M canary (this arm is funded by acq8m's PARTIAL clause: one targeted arm at the named residual subclass, never more undirected budget). Prediction-if-true: flat episodes learn the tuck while std is still high, converting the last over_current class. Prediction-if-false: flat stays a 2.64A press-up even at double exposure - exposure is not the lever, the tuck needs ref-content/phase treatment (tuck-phase anchor dose or tuck-segment curriculum), pre-register that next.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. 2M canary. Primary evidence = flat-pinned probe ON THE POD (eval_checkpoint with --cfg-set goal.rise_flat_frac=1.0 --cfg-set goal.rise_partial_frac=0.0 --cfg-set goal.rise_rsi_frac=0.0, det+sto n=6+6, DR-0): PASS = flat det>=4/6 AND sto>=4/6 valid_plant, zero over_current, h_err_end<=10mm on valid episodes, AND the standard DR-0 gate shows bridge/crouch/rsi kinds NOT regressed vs the meshref canary pair (joint valid >=8/10 non-flat episodes, valid-ep currents in the 0.7-2.2A band) -> promote: fund an 8M acquisition + port the mix into stancemix. PARTIAL = flat-pinned >=2/6 valid or over_current rate halves vs the all-fail baseline with h_err_end<10mm appearing -> exposure works but underdosed; dose flat higher (0.85) or promote to 8M with this mix. FAIL = flat-pinned still 0-1/12 valid with the same 2.64A never-tucks press-up signature -> exposure refuted as the lever; next rung is ref-content/phase treatment (tuck-phase anchor dose or tuck-segment curriculum), not more mix dosing. NOTE the twin-noise band (+-1 episode, +-1 oc term at n=12) when reading deltas; always cross-check h_err_end_mm and the video strips - counts/currents alone are gameable.

