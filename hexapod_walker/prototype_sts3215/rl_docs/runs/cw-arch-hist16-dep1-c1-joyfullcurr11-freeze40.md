# cw-arch-hist16-dep1-c1-joyfullcurr11-freeze40

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-24T09:00:15+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr9-stopcur2

**wandb_id**: yi0s9g6x

**hypothesis**: Plain English: the entire V6 stop-cert ladder (joyfullcurr6-10, 5 reward-pricing arms) never got bucket b1 (front45_20s) to certify its stop-speed bar, so buckets b2-b9 (side90/rear/full-circle -- the actual point of the operator's full-circle order) have NEVER been practiced across the whole lineage. This cycle's stopfreeze-probe (eval-only dry run, no training) proved the STRUCTURAL stop-hold lever (goal.walk_stop_freeze_s, sim_env._walk_stop_freeze_override, default off/bit-exact, 5/5 unit tests) makes the SAME stopcur2 checkpoint's b1 cert PASS outright (walkcurr/pre_b1_pass 0->1, stop_speed_m_s 0.0326->0.0133, under the 0.015 bar) with zero additional training and zero regression to progress/falls (prog_frac 1.06, falls 0). This run warm-starts real 40M-step training from stopcur2 with the SAME reward cfg (current-charge k=2.0, the proven over_current fix) plus the freeze lever on, to see whether the ladder can finally promote past b1 and actually practice b2-b9 under real training + the full joygate command mix (not just the front45 precert context), and whether the freeze's turn-in-place exemption holds up under real randomized stop/turn/resume sequences rather than the scripted precert probe.

**gate**: walkcurr/frontier must promote past b1 (reach b2 or higher) at or shortly after init, and b1's own stop cert must stay PASS through training (not regress). Standard prestage evals (DR-0 gate + own-DR + 60s joygate) must show zero NEW pathology from the freeze override itself: no new falls attributable to a stale/jerky command resumption right after a freeze release, dir_err/slip within the existing stopcur2 band, no new leg-sacrifice signature. If frontier still does not promote or the freeze introduces a new fall mode: the structural hold works at eval-time on a static probe but interacts badly with real training dynamics or the full randomized command mix -- dig into the resume-transition (frozen command -> fresh policy action) before any wider deployment of the mechanism.

