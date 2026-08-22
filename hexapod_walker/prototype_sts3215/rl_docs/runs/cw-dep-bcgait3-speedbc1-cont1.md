# cw-dep-bcgait3-speedbc1-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T00:06:48+00:00

**pod**: hexapod-mjx-train-7

**steps**: 4000000

**parent**: cw-dep-bcgait3-speedbc1

**wandb_id**: m5higxu2

**hypothesis**: Does the fast walker actually get better if we simply keep training it another 4M steps? The operator ordered this continuation because training reward was still recovering when the parent run hit its 2M budget, and wants to know whether reward keeps improving and whether that improvement is real behavior or reward hacking. This cycle's decomposition of the parent (W&B 4yitv3cc) predicts ARTIFACT: per-tick reward actually WORSENED late (-2.95 -> -3.13 across the last four windows) while ep_len_mean fell 317 -> 249 and mean pitch rose 4.7 -> 6.3 deg — the return 'recovery' from -952 to -780 is shorter episodes accumulating less of the net-negative heading+overspeed charges (heading -1.7/tick, overspeed -2.1/tick dominate), i.e. the reward currently pays the policy to fall earlier. Direction error (~78 deg) and speed (0.12 m/s) stayed command-invariant throughout. Prediction-if-operator-right: with more steps the policy escapes the charge basin, per-tick reward turns up WITH episode length, and pinned-speed panels improve. Prediction-if-artifact: rollout reward keeps 'improving' while ep_len shrinks, pitch grows, and the pinned-speed panels stay fallen/command-invariant.

**gate**: Budget 4M (+snapshots every 0.5M via --save-every). At each 1M snapshot AND at finish: pinned-speed panel 0.06/0.08/0.10 det+sto at DR-0, reporting falls, direction err, slip/m, speed_mean/prog_ratio, gait_valid, height/roll vs the parent's 2M panel (34/48 tilt_pitch falls, dir err 58-80 deg, speed 0.12-0.14 command-invariant). STOP early if rollout reward clearly plateaus or reverses over multiple 0.5M windows. VERDICT MUST decompose reward vs ep_len: quote per-tick reward (ep_rew/ep_len) and ep_len trends; reward-up-while-per-tick-and-panels-flat = MISALIGNED/reward-length artifact, report as such, fork returns to operator. Real PASS requires panel movement toward the parent gate axes (falls toward zero, dir err toward <=30, prog 0.75-1.25, slip det<=2.2/sto<=3.0). NO DOWNLOAD_ANSWER change from this run under any outcome.

