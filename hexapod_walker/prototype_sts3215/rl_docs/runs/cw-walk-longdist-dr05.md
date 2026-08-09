# cw-walk-longdist-dr05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T13:46:23+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip

**wandb_id**: nyviw5wc

**hardware_ready**: no

**hypothesis**: DR ladder (wishlist #9) re-based onto the NEW walk champion cw-walk-longdist-r2 (promoted cycle 43, md5 bcddc65c): its only weak verified axis is stochastic robustness under full model DR (DR1.0 sto 3/6 in both seeds, draw-specific stalls, no falls). One variable: same longdist config (30s, 0.05-0.06 band) trained WITH model DR 0.5 instead of --no-dr. If-true: DR0.5-trained variant keeps det slip/m <=1.1 at DR0.5 own-eval AND lifts DR1.0 sto from 3/6 to >=5/6 without det slip regressing past champion 1.24 - DR training buys robustness the DR0 lineage lacks. If-false: slip re-inflates under uncertainty (paddle worsens, like cw-walk-dr05-r1 off the old champion: det slip 1.56 with blowouts) or sto stays ~3/6 - DR training does not fix the draw-stalls and the brittleness is command-conditioning, not physics exposure. Strongest alternative: stability-pricing arms (dr05-tilt50/fall300, verdicts pending) already fix this cheaper; if they pass, this run is the A/B against them.

**gate**: Own-DR(0.5) det+sto 6/6: gait_valid 12/12, 0 term, det slip/m <=1.1; plus DR1.0 panel sto >=5/6 with det slip/m <=1.24 (baselines: r2 DR1.0 det 1.06 sto 3/6)

**verdict**: FAIL / refuted on pre-registered if-false. DR0.5-trained champion variant: own-DR0.5 leg PASSED (gv 12/12, 0 term, det agg slip/m 1.09 <=1.1, frames clean) but the decisive DR1.0 leg missed — sto 3/6, identical to parent r2's 3/6 baseline (det slip 1.15 <=1.24 ok). Stall severity softened at DR1.0 (worst sto ep prog 0.55/slip 2.94 vs parent's in-place 0.04-0.30/slip up to 27) but the DR0 panel still shows a full draw-stall (sto[4] prog 0.15, slip 10.0). Converging with tilt50 dig-in (same draws stall parent+child+seeds) and wander30 (resampling breaks stalls): draw-stalls are COMMAND-CONDITIONING, not physics exposure — DR training is not the lever. 0-c(iii) reliability front should attack command conditioning. hardware-ready: no.

