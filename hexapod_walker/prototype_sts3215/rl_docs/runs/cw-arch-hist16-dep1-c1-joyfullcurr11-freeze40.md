# cw-arch-hist16-dep1-c1-joyfullcurr11-freeze40

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-24T09:00:15+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr9-stopcur2

**wandb_id**: yi0s9g6x

**hypothesis**: Plain English: the entire V6 stop-cert ladder (joyfullcurr6-10, 5 reward-pricing arms) never got bucket b1 (front45_20s) to certify its stop-speed bar, so buckets b2-b9 (side90/rear/full-circle -- the actual point of the operator's full-circle order) have NEVER been practiced across the whole lineage. This cycle's stopfreeze-probe (eval-only dry run, no training) proved the STRUCTURAL stop-hold lever (goal.walk_stop_freeze_s, sim_env._walk_stop_freeze_override, default off/bit-exact, 5/5 unit tests) makes the SAME stopcur2 checkpoint's b1 cert PASS outright (walkcurr/pre_b1_pass 0->1, stop_speed_m_s 0.0326->0.0133, under the 0.015 bar) with zero additional training and zero regression to progress/falls (prog_frac 1.06, falls 0). This run warm-starts real 40M-step training from stopcur2 with the SAME reward cfg (current-charge k=2.0, the proven over_current fix) plus the freeze lever on, to see whether the ladder can finally promote past b1 and actually practice b2-b9 under real training + the full joygate command mix (not just the front45 precert context), and whether the freeze's turn-in-place exemption holds up under real randomized stop/turn/resume sequences rather than the scripted precert probe.

**gate**: walkcurr/frontier must promote past b1 (reach b2 or higher) at or shortly after init, and b1's own stop cert must stay PASS through training (not regress). Standard prestage evals (DR-0 gate + own-DR + 60s joygate) must show zero NEW pathology from the freeze override itself: no new falls attributable to a stale/jerky command resumption right after a freeze release, dir_err/slip within the existing stopcur2 band, no new leg-sacrifice signature. If frontier still does not promote or the freeze introduces a new fall mode: the structural hold works at eval-time on a static probe but interacts badly with real training dynamics or the full randomized command mix -- dig into the resume-transition (frozen command -> fresh policy action) before any wider deployment of the mechanism.

**verdict**: The structural stop-freeze unlocks the curriculum (first-ever promotion b0->b5; b1 stop cert holds 0.012-0.016 all run) but TRAINING with the override active damages the policy itself: held-out joygate falls regress 1/48 (stopcur2 parent) -> 7/48. OBSERVATIONS (dig-in 2x2 on the run's pod, same held-out joygate seeds 90000): A=freeze40ckpt+freeze-ON-eval 7/48 falls; B=freeze40ckpt+freeze-OFF 4/48; C=parent+OFF 1/48; D=parent+ON 2/48. B repeats A's exact DR-0 det falls (det6, det11, both over_current) with the mechanism disabled, so the DR-0 regression lives in the WEIGHTS; the eval-time mechanism on healthy parent weights costs ~1 fall (C->D, noise-band). dir_err regression is eval-mechanism-only (B dr0 33.75deg == parent 33.93; A 41.78). Videos of reproduced DR-0 falls: over_current strain terminations (det6 progressive crouch/splay, det11 upright cycling gait then current trip), not topples. INTERPRETATION (root-cause chain): falls <- over_current strain under the stop-heavy random mix <- PPO trained on corrupted stop ticks (the freeze discards the policy's action, so executed!=proposed transitions let the policy drift unpunished during holds; resume then executes a drifted action) plus b2-b5 diet shift <- applying an eval-time supervisor at TRAINING time. VERDICT: FAIL against its own gate (pre-registered if-false branch fired), NOT a lineage kill per 08-21 - mechanism repaired: new --walkcurr-cert-cfg-set applies cfg overrides to the cert/precert env ONLY (default empty = bit-exact, 24/24 tests green, tag exp/cw-arch-hist16-dep1-c1-joyfullcurr12-certfreeze). hardware-ready: no. HYPOTHESIS STATUS: half-true - bucket-promotion prediction TRUE, zero-new-pathology prediction FALSE and root-caused. Next: joyfullcurr12-certfreeze trains from stopcur2 with freeze OFF in training, ON only in the cert assay; if joygate stays in the parent band while frontier promotes, the on-policy-corruption read is confirmed and the promotion win is kept cleanly.

