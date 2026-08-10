# cw-uni-mix0-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T01:42:30+00:00

**pod**: hexapod-mjx-train-9

**steps**: 18000000

**parent**: cw-uni-mix0

**wandb_id**: eqegdii5

**hardware_ready**: False

**hypothesis**: 2nd launch attempt (1st died 0-step to fleet collision storm, gotcha 13b). Same hypothesis unchanged: ZERO walk share, pure rise/lower+hold skill acquisition off cw-uni-blend1-r2, isolating income-scale vs walk-competition.

**gate**: own-cfg DR0.5 rise/lower success >=5/6 det each by 18M steps (checked via reward curve + eval at intermediate ckpt if time allows); hold quiet (height_err_end<=8mm); VIDEO: no leg-through-floor

**verdict**: FAIL: pure walk=0.0 isolation (zero walk competition) still misses the rise/lower gate by 18M steps -- rise 0/6 the ENTIRE run (flat/bridge/crouch all 0%, incl. crouch which other lineages call solved-since-run02), lower ended 3/6 (briefly touched 6/6 near 14M step then fell back, noisy 2-ep/mode sampling). Hold got LESS quiet as training progressed: height_err_end 2-4mm early -> 33mm at the final checkpoint (real regression, not just unmet target). DR0 walk retention: gv 6/6 det+sto, video clean (6 legs cycling, no leg-through-floor, no flag leg) but slip degraded (det 2.87, sto 3.93, 1 sto termination) vs typical driving-retention bands ~1.3-1.5 -- walk quality eroded some even while walk share was 0. Completes the mix-ratio ladder (blend1-r2 walk=.7, mix40 walk=.4 FAIL, mix20 walk=.2 DIG-IN, now mix0 walk=0.0): rise/lower fails to reach gate AT EVERY MIX RATIO INCLUDING ZERO WALK COMPETITION -- cleanly refutes income-scale-vs-walk-competition as the lever. Root-cause is the open question already flagged DIG-IN via sibling cw-uni-mix20-r1; not re-flagging here, just adding the zero-competition datapoint.

