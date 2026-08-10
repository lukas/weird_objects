# cw-walk-wander120-dr05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T20:31:08+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-walk-wander-dr05

**wandb_id**: iai5ea56

**hardware_ready**: no

**hypothesis**: 2-minute continuous wandering (resample every ~5 s, 15% stops) trains through fatigue-horizon effects 60 s never sees. If-true: gait survives 120 s without drift/degradation (own-cfg gv 12/12, 0 term @120 s). If-false: long-horizon drift appears - endurance needs explicit anti-drift terms, not exposure.

**gate**: own-cfg harness det+sto 6/6 @120 s: gait_valid 12/12, 0 term, det median fwd >=4.8 m; frames watched det

**verdict**: PASS (gate intent; literal fwd-metric mis-specified). OBSERVATIONS: own-cfg DR0.5 120s det+sto gv 12/12, 0 term, prog_ratio med 0.95 det / 0.94 sto (min ep 0.889), slip/m med 1.33/1.21, along_dist_m 5.07-5.92m tracking cmd_dist_m within 2-6% every episode; DR0 retention gv 12/12, prog 0.97, slip 1.21. Frames det (own-cfg sheet + det/2 + worst sto/1 + DR0 sheet): level six-leg cycling full 120s, no flag leg/drag/late sag; height_err_end 0.4-11.5mm. Sole 'ok False' (det/2) is vel_err_mean 0.031 vs 0.030 cutoff - threshold straddle, not behavior. INTERPRETATION: gate criterion 'det median fwd>=4.8m' (=0.8 x 0.05m/s x 120s) was written as if straight-line, but harness forward_dist_m is NET start->end displacement - a random-walk quantity under wander's +/-45deg resamples/5s; PASSED parent wander60-dr05 itself made only fwd 1.56-1.65m/60s, and a simulated PERFECT tracker cannot reach 4.8m net. The intent (cover >=80% of commanded distance) is measured by along_dist_m: >=4.87m in ALL 24 eps, det med 5.62m own-cfg. VERDICT: PASS - 2-minute continuous wandering at DR0.5 holds with zero degradation, matching/beating the 60s parent's band; hardware-ready: no (inherits paddle-slip root, slip caps await contact re-baseline). HYPOTHESIS STATUS: if-true confirmed - no long-horizon drift at 120s; endurance needs no explicit anti-drift terms. Future wander gates must use along_dist_m/progress_ratio, never net forward_dist_m.

