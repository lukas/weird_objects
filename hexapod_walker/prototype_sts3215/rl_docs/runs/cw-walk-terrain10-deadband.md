# cw-walk-terrain10-deadband

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T03:43:53+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: cw-walk-terrain10

**wandb_id**: s3yj25wa

**hypothesis**: Compose: servo deadband (1.0-3.0x, the deadband30 PASS envelope) onto terrain10 (uneven-ground terrain_amp=1.0). Untried pairing -- a sluggish/dead-zone servo response could interact badly with the fine foot placement uneven terrain demands (same concern as groundtilt5-deadband, currently in flight for slopes). If-true: own-cfg det+sto 6/6 gv, 0 term, det med fwd>=1.2m; DR0 flat-no-deadband retention clean. If-false: deadband on terrain compounds into stumbles/falls the terrain-alone axis didn't show.

**gate**: Own-cfg (env.terrain_amp=1.0,env.terrain_seed=3 + dr.deadband_scale=1.0,3.0) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd>=1.2m; DR0 flat no-terrain-no-deadband retention det 6/6 gv, slip/m<=1.24, prog>=0.90; frames watched det

