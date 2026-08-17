# cw-dynrep-criticD-40m1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-17T21:40:52+00:00

**pod**: hexapod-mjx-train-7

**steps**: 40000000

**git_sha**: 8cc61c3de6309e7c9fb806a57b074047cab00ce5

**wandb_id**: 55woacy7

**hypothesis**: Give the walking robot a value function that reads the frozen pretrained dynamics transformer - the exact setup that beat from-scratch PPO on all three 1M seeds - and train a fresh command-following walker for 40M steps with joystick-style command changes and stops. Tests whether the frozen-D critic advantage compounds at scale into a download-quality command walker. Checkpoints are picked by how well the robot actually walks where commanded (progress, vx/vy+yaw tracking, slip per meter, roll, falls, slew, six-foot gait cycling), never scalar reward alone. Operator order fb_20260817T210422_9df9c7 arm B.

**gate**: Pre-registered 40M decision checkpoint, no extension without a verdict. PASS = the best-loco checkpoint on the 6-episode command-rich own-DR walk eval shows early_term_rate 0, cmd_prog_frac >= 0.6, slip_per_m <= 2.0, peak_roll_deg <= 8, contact_sw_per_s >= 3 (all six feet cycling), SCORE/loco_quality >= 10, AND walk return exceeding the 1M frozen-D cohort mean (375) at some eval point. Mechanical invariants: encoder md5 match, pred/snapshot_version pinned at 0, W&B global_step advancing, periodic+best+final checkpoints present. Rise/hold retention curves reported alongside (monitored, scratch actor - no hard retention gate). Verdict must quote the visual-quality stats (slip, roll, tracking) vs the 1M cohort numbers - a higher-reward pick with worse drag/roll is not a PASS.

**note**: Script-owned (pod_criticD40m.sh, manifest criticD40m_manifest.jsonl on-pod). Operator order fb_20260817T210422_9df9c7 arm B. Code 8cc61c3d (main). Encoder = EXACT vt2ovznc checkpoint (cw-dynrep-tf-state2-recovered1.pt, md5 9df48f687967c25085ee50171e4110ff asserted at trainer start), FROZEN as critic D throughout (no online predictor; snapshot_version must stay 0). n_envs 16 (vs cohort 8) for 40M wall clock; conservative command diversity via existing proven goal keys only: walk_cmd_resample_s=4.0 + jitter 0.5 + walk_stop_frac=0.15; no yaw command channel (proven walk stack unchanged). Checkpoints: periodic 2M (newest 3) + best by pre-registered locomotion_quality composite + final.

