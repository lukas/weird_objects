# cw-walkcurr-pf-fwd6-rscale50-sde-actbias1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-24T03:12:14+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-rscale50-sde

**hypothesis**: Plain English: the sde arm proved temporally-correlated exploration noise generates real forward excursions (the track's first: sto prog 0.32-0.47) but the robot falls forward on every one (6/6 tilt_pitch; training ~1400 tilt terms/window) and dets to a park-stand to dodge the falls -- while actbias1 proved the action-zero-point fix yields a stable, never-falling level stance but no excursions. Stack the two proven single levers: gSDE noise supplies the directional excursions, the recentered a=0 (hip+45/knee+15, a=0 = settled stance) supplies a recoverable posture underneath them, so PPO can finally reinforce a caught forward step instead of pricing every excursion as a topple. Single combination vs both parents, fresh 2M discovery. Prediction-if-true: tilt terminations collapse vs sde's ~1400/window, env/walk_freeprog_score leaves the dead band toward zero, det gate shows real stepping. Prediction-if-false-a (stable park absorbs the noise, no excursions survive): the idleterm sibling arm decides. Prediction-if-false-b (falls persist unchanged): the excursions are inherently ballistic at this noise scale and a smaller sde noise/log-std dose is the next single lever.

**gate**: Same rung-1 gate as every fwd6 arm: C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes. Mechanism-health: clip_fraction healthy; tilt-termination rate vs sde parent (~1400/window); env/walk_freeprog_score vs the [-0.10,-0.05] dead band.

