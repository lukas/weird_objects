# cw-walk-torquescale-r1-rr1-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T03:11:33+00:00

**pod**: hexapod-mjx-train-11

**steps**: 6000000

**parent**: cw-walk-torquescale

**hypothesis**: Mechanical retry of cw-walk-torquescale: died at 0 steps to the fleet's launch-collision EOFError (gotcha 13b), not a science result. Same hypothesis unchanged: isolated dr.torque_scale=0.80,1.05 off cw-walk-longdist-r2.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.torque_scale=0.80,1.05, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd>=1.2m; measure the untrained-champion baseline under the same cfg-set FIRST and require this run's worst-2 det draws to beat it outside noise; DR0 nominal retention det 6/6 gv, slip/m med<=1.24; frames watched det

