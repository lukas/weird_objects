# cw-walk-terrain10-deadband

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T03:43:53+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: cw-walk-terrain10

**wandb_id**: s3yj25wa

**hardware_ready**: False

**hypothesis**: Compose: servo deadband (1.0-3.0x, the deadband30 PASS envelope) onto terrain10 (uneven-ground terrain_amp=1.0). Untried pairing -- a sluggish/dead-zone servo response could interact badly with the fine foot placement uneven terrain demands (same concern as groundtilt5-deadband, currently in flight for slopes). If-true: own-cfg det+sto 6/6 gv, 0 term, det med fwd>=1.2m; DR0 flat-no-deadband retention clean. If-false: deadband on terrain compounds into stumbles/falls the terrain-alone axis didn't show.

**gate**: Own-cfg (env.terrain_amp=1.0,env.terrain_seed=3 + dr.deadband_scale=1.0,3.0) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd>=1.2m; DR0 flat no-terrain-no-deadband retention det 6/6 gv, slip/m<=1.24, prog>=0.90; frames watched det

**verdict**: PASS: servo deadband (1.0-3.0x) composes cleanly onto terrain10 (36mm bumps). Own-cfg det/sto prog med 1.02/1.01, slip 0.87/0.81, gv 6/6 both, 0 term -- matches parent terrain10's own band (prog 1.06/0.93) almost exactly; flat DR0 retention det 6/6 PERFECTLY clean (slip 0.85, prog 1.07, no bad draws), sto has the identical single-draw seed-4 stall (prog 0.06, slip 20.9) that parent terrain10's own flat-DR0 sto pass shows at the same index/magnitude -- a pre-existing eval-seed artifact of this checkpoint lineage, not something deadband introduced. Own-cfg det picked up that same seed-4 stall too (prog -0.01 fwd 0.18m) where parent's det pass was clean -- terrain roughness trims the margin against the known attractor a little, but video confirms it is march-in-place (six legs cycling, level body, no fall/flag-leg), not a new failure mode. Video clean on all other draws (alternating tripod, feet planted, forward progress).

