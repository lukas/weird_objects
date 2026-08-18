# probe-criticD40m1-viscompare1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-18T02:41:55+00:00

**pod**: hexapod-mjx-train-4

**hardware_ready**: False

**verdict**: KEEP best6M. Matched visual A/B (12 eps: 2 ckpts x DR0/own-DR0.3 x seeds 100/101/102, deterministic, exact trained contract, scripted fwd/+45/lateral/back/stop/restart/instant-flip @0.05m/s): best-loco @6M (md5 9b3774a4) beats periodic ck22M (md5 3d449f70) on every moving segment — fwd/diag/restart cmd_prog_frac 0.79-1.21 vs 0.48-1.11, slip_per_m ~0.9-2.1 vs 1.5-3.9, level posture (tail_roll <2.4deg, mean_h 0.13-0.14) vs ck22M lean-and-park habit (tail_roll up to 8.1deg leaning onto stiff outstretched legs in lateral/stop, walks 1cm lower, crouches). ck22M also drives the WRONG WAY on backward commands (prog_frac -0.10..-0.31 vs best6M +0.13..+0.28). SHARED gaps (both ckpts, envelope): lateral prog_frac ~0.2 w/ slip 5-11, backward broken, stop still creeps 0.02-0.06m/s, instant flip produces no reversal. 1 fall total: best6M dr0.3 s101 during back (trip-class, not a topple on video). DR0 note: env deterministic at DR0 so 3 seeds collapse to 1 distinct episode per ckpt. W&B (videos/strips/tables): https://wandb.ai/l2k2/hexapod-balance/runs/w9rfye7u ; artifacts logs/ckpt_eval/probe-criticD40m1-viscompare1/ (report.json, summary.txt, 12 MP4 + 6 side-by-side + strips); ckpt copies rl_move/sim/policies/ppo_cw_dynrep_criticD_40m1_{best6M,ck22000000}.zip. Training on train-7 untouched (23.75M and advancing at probe end).

**note**: Eval-only visual A/B probe (operator request fb_20260818T022818_d54f8e): cw-dynrep-criticD-40m1 saved best-loco checkpoint (step 6,000,000, md5 9b3774a4b119375d93598bf717498eec, saved 2026-08-17T22:55:38Z) vs newest complete periodic ck22000000 (step 22,000,000, md5 3d449f708f2a4b7e09f5a06b23fb936a, saved 2026-08-18T02:25:32Z), both from hexapod-mjx-train-7 while training continues untouched. Exact trained contract (train_ppo_transfer make_task_env walk, hist16, own goal-set walk_cmd_resample_s=4.0/jitter=0.5/stop_frac=0.15), deterministic, matched seeds 100/101/102, DR0 + own-DR0.3, fixed follow-cam MP4s + strips + side-by-side, per-segment metrics. Runner: rl_move/dynamics/eval_visual_compare.py (new, eval-only, commit 3dd931d9) on idle pod hexapod-mjx-train-4. No training, no hardware.

