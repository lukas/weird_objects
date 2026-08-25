# cw-arch-hist64-joyfullcurr13-v7-hz100-movecur1-acq1r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T06:14:35+00:00

**pod**: hexapod-mjx-train-5

**steps**: 38000000

**parent**: cw-arch-hist64-joyfullcurr13-v7-hz100-movecur1

**wandb_id**: t1l9bxzr

**hypothesis**: Plain English: at the 2M mechanism-canary budget, does k_walk_move_current=2.0 actually change anything yet? Matched-step check against the SAME recipe's uncharged reference (cw-arch-hist64-mesh-joyfullcurr13-v7-hz100-acq1) shows terminations/over_current, env/height_mm and rollout/ep_rew_mean tracking NEARLY IDENTICALLY tick-for-tick through 2M (e.g. over_current 1621/436/8/3/4... vs the reference's 1690/304/7/7/5... at the SAME global_steps) -- the charge only fires on commanded-translating ticks (s_ref>1e-3) and cmd_prog_frac stayed ~0 the whole 2M (0/4 b0_bridge promotions, matching the reference's own 0/5 at matched steps), so the charge has barely had a chance to bind. The reference's own over_current pathology didn't fully dominate until ~20-38M once it committed to a locked-leg gait -- 2M cannot distinguish 'charge prevents the relapse' from 'too early for either lineage to relapse'. This is the PARTIAL/continuation branch of the run's own pre-registered gate, not a FAIL: continuing to the SAME 40M acquisition budget the uncharged reference got, charge held constant, to see whether over_current re-emerges once cmd_prog actually moves.

**gate**: PASS: by ~38-40M, over_current termination rate + Imax stay well under the acq1/tf64-mesh-acq1 fail signature (24-38/48 held-out falls, Imax pinned 2.64-2.70A) AND walkcurr frontier/promotions move past b0 (genuine forward progress, not another frozen plateau). PARTIAL: over_current measurably lower than the matched-step acq1/tf64-mesh-acq1 trajectory but frontier still stuck at b0 -- a real but incomplete win, dig-in on the progress/current tradeoff rather than blind further extension. FAIL: over_current/Imax converge to the SAME locked-leg-tripod signature as acq1/tf64-mesh-acq1 by matched steps -- k=2.0 is insufficient against the mesh family's fully-committed exploit; escalate dose/mechanism or pursue a structural fix (k_tau_over hinge, or warm-starting the mesh lineage off a healthy primitive walker) instead of further k_walk_move_current iteration.

