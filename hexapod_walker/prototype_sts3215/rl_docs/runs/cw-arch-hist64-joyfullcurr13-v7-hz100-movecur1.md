# cw-arch-hist64-joyfullcurr13-v7-hz100-movecur1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PARTIAL

**created**: 2026-08-25T04:59:05+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-arch-hist64-joyfullcurr13-v7-hz100-gaitgate-scratch1

**wandb_id**: oo9ts5ha

**hypothesis**: Plain English: does directly pricing sustained per-servo current on WALKING ticks (reward.k_walk_move_current=2.0, bank-proven this cycle: test_walk_move_current.py, honest cycling gait beats a locked-leg fight by >40 reward and beats a stall by >10) stop the mesh-family over_current death that made BOTH the MLP (hist64-mesh-acq1) and transformer (tf64-mesh-acq1) 40M acquisitions fail identically (zero tilt_pitch, over_current every DR-0 episode, Imax pinned 2.64-2.70A, rigid 3-legs-locked/3-legs-airborne duty pattern)? This is the single-lever direct-mechanism test: SAME V7/100Hz/hist64/mesh recipe as gaitgate-scratch1 (which tried the OTHER anti-sacrifice lever, walk_gait_gate, alone -- and made the held-out fall rate WORSE, 39/48 vs the ungated 12/48) but with walk_gait_gate turned back OFF and k_walk_move_current on instead.

**gate**: PASS: training reward improves through the from-scratch valley (matching the mesh valley reference shape) AND per-tick walk_move_current_max_a stays mostly under ~2.2A by 2M (no chronic near-2.64A plateau) AND in-training eval/walk/survived_frac shows real nonzero stretches (not the near-total 0 of every prior mesh arm). PARTIAL: reward improves and over_current frequency measurably drops vs the matched-step gaitgate-scratch1/acq1 trajectories but survived_frac stays mostly 0 -- continuation candidate per 08-21. FAIL: over_current termination rate and Imax profile are statistically indistinguishable from gaitgate-scratch1/hist64-mesh-acq1 at matched steps -- the charge is too weak/mis-thresholded, escalate to a higher k or lower threshold, or the mechanism itself is insufficient and needs a structural fix (e.g. reduce required stance duty via gait-phase shaping).

**verdict**: Result: PARTIAL (continuation, not FAIL) -- 2M is too early to judge k_walk_move_current=2.0 at all. Matched-step comparison against the SAME recipe uncharged reference (cw-arch-hist64-mesh-joyfullcurr13-v7-hz100-acq1) shows terminations/over_current, env/height_mm and rollout/ep_rew_mean tracking NEARLY IDENTICALLY tick-for-tick through 2M (over_current e.g. 1621/436/8/3/4/6/7... at steps 786k-1327k vs the references own 1690/304/7/7/5/8/7... at the SAME steps; ep_rew -623 vs references -573 at 2015232). walkcurr/b0_bridge_10s/cmd_prog_frac stayed ~0 all 4 cert rounds (0/4 promotions), matching the references own 0/5 at matched steps -- the charge only fires on commanded-translating ticks (s_ref>1e-3) and the policy essentially never reaches that condition this early, so it has not had a chance to bind yet. DR-0 walk_det_0 video (60s, deterministic) confirms: body stays in the same wide-splayed static crouch the whole episode, no forward translation, no six-leg cycling -- but this matches what an UNTRAINED-THIS-FAR mesh/100Hz walkcurr policy looks like generically (same generic early-valley shape the gate itself names as the PASS-shape reference), not a charge-specific stall. The references OWN over_current pathology did not fully dominate until ~20-38M once it committed to a locked-leg gait, so this 2M canary genuinely cannot distinguish charge-prevents-relapse from too-early-for-either-lineage-to-relapse. This is the runs own pre-registered PARTIAL/continuation branch (08-21 ruling: reward/metrics matching a KNOWN-to-recover reference shape, not a flat/failed signal, is a continue case). Next: continuing to the SAME 38M-more/40M-total acquisition budget the uncharged acq1 got, charge held constant, launched this cycle as cw-arch-hist64-joyfullcurr13-v7-hz100-movecur1-acq1r2 (VERIFIED RUNNING train-5) -- will show whether over_current re-emerges once cmd_prog actually moves past ~0, which is the real test of this lever.

